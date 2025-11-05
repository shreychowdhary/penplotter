import pigpio
from time import sleep
import math
import json
import sys
import signal

class PenPlotter:
    def __init__(self, settings_file):
        f = open(settings_file)
        data = json.load(f)

        self.width = data["width"]
        self.left0 = data["left0"]
        self.right0 = data["right0"]
        self.offset_x = data["offset_x"]
        self.offset_y = data["offset_y"]
        self.rot_dist = data["rot_dist"]
        self.rot_steps = data["rot_steps"]
        self.max_line_length = data["max_line_length"]
        self.left_direction_pin = data["left_direction_pin"]
        self.left_step_pin = data["left_step_pin"]
        self.left_enable_pin = data["left_enable_pin"]
        self.right_direction_pin = data["right_direction_pin"]
        self.right_step_pin = data["right_step_pin"]
        self.right_enable_pin = data["right_enable_pin"]
        self.servo_pin = data["servo_pin"]
        self.pen_down_speed = data["pen_down_speed"]
        self.pen_up_speed = data["pen_up_speed"]
        self.servo_down_speed = data["servo_down_speed"]
        self.servo_up_speed = data["servo_up_speed"]

        self.x0 = (self.width**2 - self.right0**2 + self.left0**2)/(2*self.width)
        self.y0 = math.sqrt(self.left0**2-self.x0**2)
        self.left_roundoff, self.right_roundoff = 0,0
        self.left_step_sum, self.right_step_sum = 0,0
        self.cur_x = 0
        self.cur_y = 0
        self.is_pen_down = False
        self.pi = pigpio.pi()

        self.pi.set_mode(self.left_direction_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.left_step_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.left_enable_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.right_direction_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.right_step_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.right_enable_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.servo_pin, pigpio.OUTPUT)
        self.pi.write(self.left_enable_pin, 0)
        self.pi.write(self.right_enable_pin, 0)
        self.running = False

    def relative_go_to(self, delta_x, delta_y):
        self.go_to(self.cur_x+delta_x, self.cur_y+delta_y)

    def calculate_steps(self, cur_x, cur_y, x, y):
        delta_left = math.sqrt((x+self.x0)**2+(-y+self.y0)**2) - math.sqrt((cur_x+self.x0)**2+(-cur_y+self.y0)**2)
        delta_right = math.sqrt((self.width-(x+self.x0))**2+(-y+self.y0)**2) - math.sqrt((self.width-(cur_x+self.x0))**2+(-cur_y+self.y0)**2)
        left_steps = delta_left/self.rot_dist * self.rot_steps
        right_steps = delta_right/self.rot_dist * self.rot_steps
        return left_steps, right_steps

    def go_to(self, target_x, target_y, pen_down=False):
        orig_x = self.cur_x
        orig_y = self.cur_y
        dist = math.sqrt((target_x-orig_x)**2 + (target_y-orig_y)**2)
        pen_speed = self.pen_down_speed if pen_down else self.pen_up_speed
        num_segments = math.ceil(dist/self.max_line_length)
        num_segments = num_segments if pen_down else 1
        for i in range(num_segments):
            x = (target_x-orig_x)*(i+1)/num_segments + orig_x 
            y = (target_y-orig_y)*(i+1)/num_segments + orig_y
            left_steps, right_steps = self.calculate_steps(self.cur_x, self.cur_y, x, y)
            self.left_roundoff += left_steps - int(left_steps)
            self.right_roundoff += right_steps - int(right_steps)
            left_steps = int(left_steps)
            right_steps = int(right_steps)
            
            if abs(self.left_roundoff) >= 1:
                left_steps += 1 if self.left_roundoff > 0 else -1
                self.left_roundoff -= 1 if self.left_roundoff > 0 else -1
            if abs(self.right_roundoff) >= 1:
                right_steps += 1 if self.right_roundoff > 0 else -1
                self.right_roundoff -= 1 if self.right_roundoff > 0 else -1
            self.left_step_sum += left_steps
            self.right_step_sum += right_steps

            left_dir = left_steps > 0 
            right_dir = right_steps < 0
            left_steps = abs(left_steps)
            right_steps = abs(right_steps)
            self.pi.write(self.left_direction_pin, left_dir)
            self.pi.write(self.right_direction_pin, right_dir)

            combined_steps = min(left_steps, right_steps)
            steps_remaining = max(left_steps, right_steps) - combined_steps

            for i in range(combined_steps):
                self.pi.write(self.left_step_pin, 1)
                self.pi.write(self.right_step_pin, 1)
                sleep(pen_speed)
                self.pi.write(self.left_step_pin, 0)
                self.pi.write(self.right_step_pin, 0)
                sleep(pen_speed)
            
            step_pin = self.left_step_pin if left_steps > right_steps else self.right_step_pin
            for i in range(steps_remaining):
                self.pi.write(step_pin, 1)
                sleep(pen_speed)
                self.pi.write(step_pin, 0)
                sleep(pen_speed)
            self.cur_x = x
            self.cur_y = y
        
    def calculate_pos_from_steps(self, left_steps, right_steps):
        delta_left = (left_steps/self.rot_steps)*self.rot_dist
        delta_right = (right_steps/self.rot_steps)*self.rot_dist
        
        left_length = delta_left + self.left0
        right_length = delta_right + self.right0
        x = (self.width**2+left_length**2-right_length**2)/(2*self.width)
        y = math.sqrt(right_length**2-(self.width-x)**2)
        return x-self.x0,y-self.y0

    def pen_down(self):
        if self.is_pen_down:
            return
        for i in range(81):
            self.pi.set_servo_pulsewidth(self.servo_pin, i*10+1000)
            sleep(self.servo_down_speed)
        self.is_pen_down = True

    def pen_up(self):
        if not self.is_pen_down:
            return
        for i in range(81):
            self.pi.set_servo_pulsewidth(self.servo_pin, 1800-i*10)
            sleep(self.servo_up_speed)
        self.is_pen_down = False

    def run_gcode(self, gcode_file):
        self.running = True
        self.left_roundoff = 0
        self.right_roundoff = 0
        file = open(gcode_file)
        i = 0
        for line in file:
            if not self.running:
                break
            i += 1
            # Get rid of gcode comments
            line = line.split(";")[0]
            tokens = line.split()
            if len(tokens) == 0:
                continue
            command = tokens[0]
            if command == "G21":
                pass
                # Using mm, which is assumed
            elif command == "G90":
                pass
                #absolute position
            elif command == "G0":
                self.pen_up()
                x = float(tokens[1][1:])+self.offset_x
                y = float(tokens[2][1:])+self.offset_y
                print("G0", x, y, i)
                self.go_to(x, y, pen_down=False)
            elif command == "G1":
                self.pen_down() 
                x = float(tokens[1][1:])+self.offset_x
                y = float(tokens[2][1:])+self.offset_y
                #print("G1", x, y, i)
                self.go_to(x,y, pen_down=True)
            elif command == "M2":
                #end program
                break
            else:
                print("UNKNOWN", command)

        self.pen_up()
        self.go_to(0,0, pen_down=True)
        print("DONE")


    def __del__(self):
        self.pi.stop()
    
    def signal_handler(self, signum, frame):
        self.running = False
        print("Ending")

    def estimate_time(self, gcode_file):
        file = open(gcode_file)
        i = 0
        cur_x, cur_y = 0,0
        total_time = 0
        pen_up = True
        for line in file:
            # Get rid of gcode comments
            line = line.split(";")[0]
            tokens = line.split()
            if len(tokens) == 0:
                continue
            command = tokens[0]
            if command == "G21":
                pass
                # Using mm, which is assumed
            elif command == "G90":
                pass
                #absolute position
            elif command == "G0":
                if not pen_up:
                    total_time += 80*self.servo_up_speed
                x = float(tokens[1][1:])+self.offset_x
                y = float(tokens[2][1:])+self.offset_y
                dist = math.sqrt((cur_x-x)**2+(cur_y-y)**2)
                left_steps, right_steps = self.calculate_steps(cur_x, cur_y, x, y)
                steps = max(left_steps, right_steps)
                total_time += steps * 2 * self.pen_up_speed
                cur_x,cur_y = x,y
                pen_up = True
            elif command == "G1":
                if pen_up:
                    total_time += 80*self.servo_down_speed
                target_x = float(tokens[1][1:])+self.offset_x
                target_y = float(tokens[2][1:])+self.offset_y
                dist = math.sqrt((cur_x-target_x)**2+(cur_y-target_y)**2)
                orig_x, orig_y = cur_x, cur_y
                num_segments = math.ceil(dist/self.max_line_length)
                for i in range(num_segments):
                    x = (target_x-orig_x)*(i+1)/num_segments + orig_x 
                    y = (target_y-orig_y)*(i+1)/num_segments + orig_y
                    left_steps, right_steps = self.calculate_steps(cur_x, cur_y, x, y)
                    self.left_roundoff += left_steps - int(left_steps)
                    self.right_roundoff += right_steps - int(right_steps)
                    left_steps = int(left_steps)
                    right_steps = int(right_steps)
                    
                    if abs(self.left_roundoff) >= 1:
                        left_steps += 1 if self.left_roundoff > 0 else -1
                        self.left_roundoff -= 1 if self.left_roundoff > 0 else -1
                    if abs(self.right_roundoff) >= 1:
                        right_steps += 1 if self.right_roundoff > 0 else -1
                        self.right_roundoff -= 1 if self.right_roundoff > 0 else -1

                    left_steps = abs(left_steps)
                    right_steps = abs(right_steps)
                    steps = max(left_steps, right_steps)
                    total_time += steps * 2 * self.pen_down_speed
                    cur_x, cur_y = x, y
                pen_up = False
            elif command == "M2":
                #end program
                break
            else:
                print("UNKNOWN", command)
        print("Estimated Time: ", total_time/60)
    

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 penplotter.py <settings.json> <gcode>")
        exit()
    pen_plotter = PenPlotter(sys.argv[1])
    signal.signal(signal.SIGINT, pen_plotter.signal_handler)
    pen_plotter.estimate_time(sys.argv[2])
    pen_plotter.run_gcode(sys.argv[2])
