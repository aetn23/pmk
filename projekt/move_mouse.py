import sys
import mouse

def main():
    stm32_dev = open("/dev/ttyACM0", "r")
    for line in stm32_dev:
        if (line[0] == '0' or line[0] == '1') and line.__len__ == :
            val = int(line[0:8], 2)
            bytes = val.to_bytes(1, byteorder=sys.byteorder, signed=False)
            final_val = int.from_bytes(bytes, byteorder=sys.byteorder, signed=True);
            print(line[8] + ': ' + str(final_val))
            x_val = 0 
            y_val = 0
            if final_val > 0 and final_val < 8:
                final_val = 0
#            final_val *= 2
            if line[8] == 'x':
                x_val = final_val
            else:
                y_val = final_val
            mouse.move(x_val, y_val, absolute=False, duration=0);

if __name__ == "__main__":
    main()
