# Method 1 (Using the manual steps)
def decimal_to_hexadecimal(decimal_num):
    hex_characters = "0123456789ABCDEF"
    if decimal_num == 0:
        return "0"
    
    hexadecimal = ""
    while decimal_num > 0:
        remainder = decimal_num % 16
        hexadecimal = hex_characters[remainder] + hexadecimal
        decimal_num //= 16
    
    return hexadecimal

decimal_number = 13
hexadecimal_value = decimal_to_hexadecimal(decimal_number)
print("Manual steps output: ", hexadecimal_value)

# Method 2 (Using the hex function)
decimal_number = 1000
hexadecimal_value = hex(decimal_number)
print("Hex function output: ", hexadecimal_value)