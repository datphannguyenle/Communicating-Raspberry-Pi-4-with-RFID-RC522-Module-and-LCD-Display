Here's the translation of the converted guide into English:

---

# Communicating Raspberry Pi 4 with RFID-RC522 Module and LCD Display

## Authors

- TA: Le Phan Nguyen Dat
- Instructor: MEng. Truong Quang Phuc

## Hardware
- Raspberry Pi 4 Model B
- RFID-RC522 Module
- 16x2 LCD with I2C Module

## Connections

### RFID-RC522 Module
| RFID-RC522 Pin | Raspberry Pi Pin         |
|----------------|--------------------------|
| VCC            | 3.3V (Pin 1)             |
| GND            | GND (Pin 6)              |
| MISO           | GPIO 9 (MISO, Pin 21)    |
| MOSI           | GPIO 10 (MOSI, Pin 19)   |
| SCK            | GPIO 11 (SCLK, Pin 23)   |
| SDA            | GPIO 8 (CE0, Pin 24)     |
| RST            | GPIO 25 (Pin 22)         |

### 16x2 LCD with I2C Module
| I2C Module Pin | Raspberry Pi Pin         |
|----------------|--------------------------|
| VCC            | 5V (Pin 4)               |
| GND            | GND (Pin 9)              |
| SDA            | GPIO 2 (SDA, Pin 3)      |
| SCL            | GPIO 3 (SCL, Pin 5)      |

## Enable SPI Interface
1. Open the Raspberry Pi configuration tool:
    ```bash
    sudo raspi-config
    ```

2. Navigate to `Interfacing Options` using the arrow keys and press `Enter`.

3. Select `SPI` and press `Enter`.

4. When asked if you want to enable SPI, select `Yes` and press `Enter`.

5. Exit the `raspi-config` tool by navigating to `Finish` and pressing `Enter`.

6. Reboot your Raspberry Pi to apply the changes:
    ```bash
    sudo reboot
    ```

## Create a Virtual Environment
1. Ensure you have the necessary packages to create a virtual environment:
    ```bash
    sudo apt-get install python3-venv
    ```

2. Create a virtual environment:
    ```bash
    python3 -m venv myenv
    ```

3. Activate the virtual environment:
    ```bash
    source myenv/bin/activate
    ```

## Install Python Libraries
1. Update the package lists:
    ```bash
    sudo apt-get update
    ```

2. Install the required libraries within the virtual environment:
    ```bash
    sudo apt-get install -y python3-smbus i2c-tools python3-spidev
    pip install mfrc522 RPi.GPIO
    ```

## Create and Run Code
1. Create a Python file using SCP or nano:
    ```bash
    nano rfid_lcd.py
    ```

2. Copy the following code into the file:
    ```python
    import RPi.GPIO as GPIO
    from mfrc522 import SimpleMFRC522
    import smbus2
    import time

    # Initialize the RFID device
    reader = SimpleMFRC522()

    # I2C parameters for LCD
    I2C_ADDR = 0x27  # I2C device address
    LCD_WIDTH = 16   # Maximum characters per line

    # LCD constants
    LCD_CHR = 1  # Sending data
    LCD_CMD = 0  # Sending command
    LCD_LINE_1 = 0x80  # LCD RAM address for the 1st line
    LCD_LINE_2 = 0xC0  # LCD RAM address for the 2nd line
    LCD_BACKLIGHT = 0x08  # On

    ENABLE = 0b00000100  # Enable bit

    # Timing constants
    E_PULSE = 0.0005
    E_DELAY = 0.0005

    # Open I2C interface
    try:
        bus = smbus2.SMBus(1)  # Rev 2 Pi uses bus 1
    except Exception as e:
        print(f"Error initializing I2C bus: {e}")
        sys.exit(1)

    def lcd_init():
        lcd_byte(0x33, LCD_CMD)  # Initialize
        lcd_byte(0x32, LCD_CMD)  # Initialize
        lcd_byte(0x06, LCD_CMD)  # Cursor move direction
        lcd_byte(0x0C, LCD_CMD)  # Display On, Cursor Off, Blink Off
        lcd_byte(0x28, LCD_CMD)  # Data length, number of lines, font size
        lcd_byte(0x01, LCD_CMD)  # Clear display
        time.sleep(E_DELAY)

    def lcd_byte(bits, mode):
        bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
        bits_low = mode | ((bits << 4) & 0xF0) | LCD_BACKLIGHT

        # High bits
        bus.write_byte(I2C_ADDR, bits_high)
        lcd_toggle_enable(bits_high)

        # Low bits
        bus.write_byte(I2C_ADDR, bits_low)
        lcd_toggle_enable(bits_low)

    def lcd_toggle_enable(bits):
        time.sleep(E_DELAY)
        bus.write_byte(I2C_ADDR, (bits | ENABLE))
        time.sleep(E_PULSE)
        bus.write_byte(I2C_ADDR, (bits & ~ENABLE))
        time.sleep(E_DELAY)

    def lcd_string(message, line):
        message = message.ljust(LCD_WIDTH, " ")

        lcd_byte(line, LCD_CMD)

        for i in range(LCD_WIDTH):
            lcd_byte(ord(message[i]), LCD_CHR)

    def lcd_clear():
        lcd_byte(0x01, LCD_CMD)

    if __name__ == '__main__':
        try:
            lcd_init()
            while True:
                print("Waiting for card scan...")
                id, text = reader.read()

                print(f"ID: {id}")
                print(f"Content: {text}")

                lcd_string(f"ID: {id}", LCD_LINE_1)
                lcd_string(text.strip(), LCD_LINE_2)

                time.sleep(2.0)

        except KeyboardInterrupt:
            print("Measurement stopped by User")
        except Exception as e:
            print(f"An error occurred: {e}")
        finally:
            lcd_clear()
            GPIO.cleanup()
    ```

3. Save and exit the file by pressing `Ctrl+X`, then `Y`, and `Enter`.

4. Make the file executable (optional but recommended):
    ```bash
    chmod +x rfid_lcd.py
    ```

5. Run the LCD program:
    ```bash
    python3 rfid_lcd.py
    ```

6. Remember to activate your virtual environment before running the script if it is not already activated:
    ```bash
    source myenv/bin/activate
    ```

To deactivate the virtual environment, you can use:
```bash
deactivate
```
