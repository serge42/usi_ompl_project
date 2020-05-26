# Compilation
clang++ -std=c++11 -Wall -Werror -g -c -I/opt/local/include -I/opt/local/include/eigen3 test.cpp -o test.o

# Linking
clang++ -L/opt/local/lib -lompl -lompl_app -lompl_app_base test.o -o test