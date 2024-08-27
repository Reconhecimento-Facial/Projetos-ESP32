void lcd_init(void); // Initialize the LCD
void lcd_send_cmd(char cmd); // Send a command to the LCD
void lcd_send_data(char data); // Send data to the LCD
void lcd_send_string(char *str); // Send a string to the LCD
void lcd_put_cur(int row, int col); // Set the cursor position
void lcd_clear(void); // Clear the LCD