""" ARAP Webots main file """
import robot
import cv2 as cv
import numpy as np
import time

def main():
    # Initialize robot
    epuck = robot.ARAP()
    epuck.init_devices()

    # Dictionary to keep track of which colors have ever been seen
    seen = {
        "red": False,
        "green": False,
        "blue": False
    }
    
    # Horse detection variables
    horse_saved = False
    horse_detection_counter = 0
    horse_detection_interval = 5

    while True:
        # Reset actuators and read sensors
        epuck.reset_actuator_values()
        distance_range = epuck.get_sensor_input()
        epuck.blink_leds()

        # Get average RGB values from camera
        avg_red, avg_green, avg_blue = epuck.get_camera_image(5)

        detected_color = None

        # --- Simple Color Detection ---
        if avg_red > avg_green and avg_red > avg_blue and avg_red > 100:
            detected_color = "red"
        elif avg_green > avg_red and avg_green > avg_blue and avg_green > 100:
            detected_color = "green"
        elif avg_blue > avg_red and avg_blue > avg_green and avg_blue > 135:
            detected_color = "blue"  # higher threshold to avoid sky color

        # --- Respond to Detected Color ---
        if detected_color:
            print(f"I see {detected_color}")

            # If it is the first time, update history
            if not seen[detected_color]:
                seen[detected_color] = True

            # Show a summary of all previously seen colors
            summary = [color for color, has_been_seen in seen.items() if has_been_seen]
            print("Previous colors:", ", ".join(summary))

        # --- Horse Detection ---
        horse_detection_counter += 1
        if horse_detection_counter >= horse_detection_interval:
            horse_detection_counter = 0
            
            # Get raw camera image for OpenCV processing
            camera = epuck.camera
            width = camera.getWidth()
            height = camera.getHeight()
            image = camera.getImage()
            
            # Convert to numpy array and process
            img_array = np.frombuffer(image, dtype=np.uint8).reshape((height, width, 4))
            img_bgr = img_array[:, :, :3]  # Use only BGR channels
            
            # Detect horse (brown regions)
            mask = (
                (img_bgr[:,:,2] > 80) & (img_bgr[:,:,2] < 180) &  # R channel
                (img_bgr[:,:,1] > 30) & (img_bgr[:,:,1] < 100) &  # G channel
                (img_bgr[:,:,0] < 70)                         # B channel
            )
            frac = mask.sum() / float(height * width)  # Fraction of brown pixels
            
            if frac > 0.05 and not horse_saved:
                print("Horse detected! Saving image...")
                # Save a small snapshot
                resized = cv.resize(img_bgr, (32, 32))
                cv.imwrite("Horse_detected.png", resized)
                horse_saved = True
                
                # Stop and wait for 3 seconds
                epuck.speeds[epuck.LEFT] = 0
                epuck.speeds[epuck.RIGHT] = 0
                epuck.set_actuators()
                epuck.step()
                time.sleep(3)
                
                # Back + Right to avoid horse
                epuck.move_backward()
                epuck.turn_right()

        # --- Movement Logic ---
        if epuck.front_obstacles_detected():
            epuck.move_backward()
            epuck.turn_left()
        else:
            epuck.run_braitenberg()

        # Apply motor speeds and LED states
        epuck.set_actuators()
        epuck.step()

if __name__ == "__main__":
    main()
