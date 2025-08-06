#include <string>
#include <iostream>

int convert_hex_to_dec(const std::string &num) {
  int suma = 0;
  for (size_t i = 0; i < num.length(); i++) {
    if (num[i] >= 65) {
      suma = suma * 16 + num[i] - 65 + 10;
    } else {
      suma = suma * 16 + num[i] - 48;
    }
  }
  return suma;
}

int main() {
  std::string num = "\x02\x31\x03";
  std::cout << num.length() << std::endl;

  num.erase(num.find("\x03")).erase(num.find("\x02"), 1);

  std::cout << num.length() << std::endl;

  int a = convert_hex_to_dec(num);
  std::cout << a << std::endl;
  return 0;
}