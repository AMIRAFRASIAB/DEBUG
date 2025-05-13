# DEBUG

A simple serial debugging utility for embedded C projects.

## How to Use

1. **Clone the Repository**

   ```bash
   git clone https://github.com/AMIRAFRASIAB/DEBUG.git
   ```

2. **Add Source File**

   - Add `serial_debugger.c` from the `src` folder to your compile path.

3. **Add Include Path**

   - Add all files from the `Inc` folder to your include path.

4. **Setup Driver Files**

   - Copy the `driver` folder **outside** the Git project.
   - Add all `.h` files from that folder to your include path.
   - Add `hmi.c` from that folder to your compile path.

5. **Configure for Your MCU**

   - Modify the config and driver files based on your microcontroller.

6. **Initialize the engine**

   - Place `debug_init()` API function inside the main

7. **Usage**
   - Include `"serial_debugger.h"` file every where you need to use the log APIs
   - Use `LOG_TRACE(...)`, `LOG_INFO(...)`, `LOG_WARNING(...)`, `LOG_ERROR(...)`, `LOG_FATAL(...)` to print your text over UART
   - Treat upper macros like standard `printf(...)` function    

## License

This project is licensed under the MIT License.
