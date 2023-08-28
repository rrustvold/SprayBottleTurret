import asyncio
import requests
import time
from datetime import datetime, timedelta

import board
import busio
import numpy as np
import adafruit_mlx90640

import matplotlib
matplotlib.use('Agg')

import matplotlib.pyplot as plt

from flask import Flask, request, send_file
import RPi.GPIO as GPIO

from adafruit_motorkit import MotorKit
from adafruit_motor import stepper
import board

import cv2


kit = MotorKit(i2c=board.I2C())
fire_flag = False
# The estimated state of the turret's direction
angle = 0
is_rotating = False
last_pic_time = datetime.now()

def create_app(test_config=None):
    
    async def sound():
        global fire_flag
        global is_rotating

        GPIO.output(SOUND_LED_PIN, GPIO.HIGH)
        if config["fire_on_sound"] is True:
            if fire_flag is False and is_rotating is False:
                fire()
            else:
                await asyncio.sleep(1)
        elif (
            config["fire_on_sound_and_motion"] is True and
            GPIO.input(PIR_DETECT_PIN)
        ):
            print(f"{fire_flag=}")
            if fire_flag is False and is_rotating is False:
                fire()
            else:
                await asyncio.sleep(1)
        else:
            await asyncio.sleep(1)
        GPIO.output(SOUND_LED_PIN, GPIO.LOW)
    
    def sound_detect_callback(*args):
        asyncio.run(sound())
        
    async def motion():
        if GPIO.input(PIR_DETECT_PIN):
            GPIO.output(PIR_LED_PIN, GPIO.HIGH)
            print("target acquired")
            if config["fire_on_motion"] is True:
                if fire_flag is False:
                    fire()
                else:
                    await asyncio.sleep(1)
        else:
            GPIO.output(PIR_LED_PIN, GPIO.LOW)

    def pir_detect_callback(*args):
        asyncio.run(motion())
	
    
    app = Flask(__name__, instance_relative_config=True)

    SLEEP_PIN = 22
    DIRECTION_PIN = 23
    SOUND_DETECT_PIN = 4
    PIR_DETECT_PIN = 17
    SOUND_LED_PIN = 27
    PIR_LED_PIN = 18


    # Hardware Set up
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SLEEP_PIN, GPIO.OUT)
    GPIO.output(SLEEP_PIN, GPIO.LOW)
    GPIO.setup(DIRECTION_PIN, GPIO.OUT)
    GPIO.setup(SOUND_DETECT_PIN, GPIO.IN)
    GPIO.setup(PIR_DETECT_PIN, GPIO.IN)
    GPIO.setup(SOUND_LED_PIN, GPIO.OUT)
    GPIO.setup(PIR_LED_PIN, GPIO.OUT)
    
    GPIO.add_event_detect(
        SOUND_DETECT_PIN,
        GPIO.RISING, 
        callback=sound_detect_callback, 
        bouncetime=1000
    )
    
    GPIO.add_event_detect(
        PIR_DETECT_PIN,
        GPIO.BOTH, 
        callback=pir_detect_callback, 
        bouncetime=1000
    )
    
    # Camera stuff
    i2c = busio.I2C(board.SCL, board.SDA, frequency=400000) # setup I2C
    mlx = adafruit_mlx90640.MLX90640(i2c) # begin MLX90640 with I2C comm
    mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_2_HZ # set refresh rate
    mlx_shape = (24,32)
    

    # App configuration
    config = {
        "fire_on_sound": False,
        "fire_on_motion": False,
        "fire_on_sound_and_motion": False,
        "fire_length": 200,  # ms
        "is_heat_seaking": False,
    }
    
    HOME_ASSISTANT_FIRE_CALLBACK = "http://192.168.4.103:8123/api/webhook/-Pb-hrfOCjj8f3n7rwqK-Jq8t"
    HOME_ASSISTANT_ANGLE_ESTIMATE_CALLBACK = "http://192.168.4.103:8123/api/webhook/-P6DXFedyRM7l6ivZM0T6krx7"
    
    def fire(length: int = None):
        global fire_flag
        if fire_flag is True:
            time.sleep(1)
            return
        
        fire_flag = True
        if length is None:
            length = config["fire_length"]
            
        print("FIRE!")
        GPIO.output(DIRECTION_PIN, GPIO.HIGH)
        GPIO.output(SLEEP_PIN, GPIO.HIGH)
        # Should this block?
        print(f"for {length/1000}")
        time.sleep(length/1000)
        print("reload")
        GPIO.output(SLEEP_PIN, GPIO.LOW)
        fire_flag = False
        requests.post(HOME_ASSISTANT_FIRE_CALLBACK)

    @app.route("/fire")
    def fire_endpoint():
        fire_length = int(request.args.get("length", 200))
        fire(fire_length)
        return "<p> Pew pew </p>"
        
    
    @app.route("/rotate")
    def rotate_endpoint():
        theta = float(request.args.get("theta", 0))
        
        asyncio.run(
            rotate(theta, release=request.args.get("release", True))
        )
        
        return f"{angle}"
        
    @app.route("/goto")
    def go_to():
        theta = float(request.args.get("theta", 0))
        global angle
        diff = theta - angle
        asyncio.run(
            rotate(diff, release=request.args.get("release", True))
        )
        
        return "angle"
        
    @app.route("/release")
    def release():
        kit.stepper1.release()
        
    @app.route("/hold")
    def hold():
        kit.stepper1.onestep(style=stepper.DOUBLE)
        
    @app.route("/home")
    def home():
        global angle
        angle = 0
        report_angle()
        return "home"
        
    def report_angle():
        global angle
        requests.post(
            HOME_ASSISTANT_ANGLE_ESTIMATE_CALLBACK, 
            data={"angle": angle},
        )
        
    async def rotate(theta_deg, release=True):
        
        global angle
        angle += theta_deg
        global is_rotating
        is_rotating = True
        # 200 steps per revolution
        if theta_deg >= 0:
            direction = stepper.FORWARD
        else:
            theta_deg *= -1
            direction = stepper.BACKWARD
            
        # 22 teeth on outer ring gear. 8 teeth on pinion
        gearing = 22/8
            
        num_steps = 200 * (theta_deg / 360) * gearing
        
        for i in range(int(num_steps)):
            kit.stepper1.onestep(style=stepper.DOUBLE, direction=direction)
            time.sleep(0.0051)
            
        if release:
            kit.stepper1.release()
            
        report_angle()
        # await asyncio.sleep(2)
        is_rotating = False
        
    @app.route("/thermal-image")
    def thermal_image():
        global last_pic_time
        image = "snapshot.png"
        if (
            request.args.get("new") 
            and not config["is_heat_seaking"]
            and datetime.now() - last_pic_time > timedelta(seconds=10)
        ):
            last_pic_time = datetime.now()
            asyncio.run(take_picture(image, True))
        
        return send_file(image, mimetype="image/png")
        
    
    @app.route("/heat-seaking")
    def heat_seaking():
        config["is_heat_seaking"] = bool(request.args.get("on", False))
        if config["is_heat_seaking"] is True:
            asyncio.run(heat_seaking())
            print("hello")
            
        return f"{config['is_heat_seaking']=}"
            
    async def heat_seaking():
        while config["is_heat_seaking"]:
            error = await take_picture(None, False)
            # about 15 degrees per 4.5 pixels
            error_degrees = error * 15 / 4.5
            print(f"{error_degrees=}")
            # await asyncio.sleep(2)
            await rotate(error_degrees)
        
    async def take_picture(filename, save=False):
        print(f"Taking new picture {datetime.now()}")
        # setup the figure for plotting
        fig, ax = plt.subplots(figsize=(12,7))
        therm1 = ax.imshow(np.zeros(mlx_shape), vmin=0, vmax=60) #start plot with zeros
        cbar = fig.colorbar(therm1) # setup colorbar for temps
        cbar.set_label('Temperature [$^{\circ}$C]', fontsize=14) # colorbar label

        frame = np.zeros((24*32,)) # setup array for storing all 768 temperatures
        
        
        
        while True:
            try:
                # read MLX temperatures into frame var
                mlx.getFrame(frame) 
                
                
                
                # reshape to 24x32
                data_array = np.flipud(np.reshape(frame, mlx_shape)) 
                
                
                
                therm1.set_data(data_array)
                therm1.set_clim(
                    vmin=np.min(data_array), vmax=np.max(data_array)
                )
                # update colorbar range
                cbar.update_normal(therm1) 
                
                binary_array = np.copy(data_array)
                
                # Set the upper right and lower left corners to 0 since they are always hot
                binary_array[0][31] = 0
                binary_array[23][31] = 0
                percentile = np.percentile(frame, 99)
                for i in range(24):
                    for j in range(32):
                        if binary_array[i][j] < percentile:
                            binary_array[i][j] = 0
                            
                M = cv2.moments(binary_array)
                cX = int(M["m10"] / M["m00"])
                print(cX)

                error = 16 - cX
            except ValueError:
                pass
            else:
                break

        if save:
            fig.savefig(
                f"motor/{filename}", 
                dpi=300, 
                facecolor='#FCFCFC',
                bbox_inches='tight'
            )
        
        plt.close(fig)
             
        return error
        
                
    
	    
    @app.route("/config", methods=["PATCH", "GET"])
    def config_endpoint():
        if request.method == "PATCH":
            config.update(request.form)
        elif request.method == "GET":
            param = int(request.args.get("mode", 0))
            if param == 0:
                config.update(
                    {
                        "fire_on_sound": False,
                        "fire_on_motion": False,
                        "fire_on_sound_and_motion": False,
                    }
                )
            elif param == 1:
                config.update(
                    {
                        "fire_on_sound": True,
                        "fire_on_motion": False,
                        "fire_on_sound_and_motion": False,
                    }
                )
            elif param == 2:
                config.update(
                    {
                        "fire_on_sound": False,
                        "fire_on_motion": True,
                        "fire_on_sound_and_motion": False,
                    }
                )
            elif param == 3:
                config.update(
                    {
                        "fire_on_sound": False,
                        "fire_on_motion": False,
                        "fire_on_sound_and_motion": True,
                    }
                )
                
        return config
	    
    return app
