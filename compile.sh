# Compilation
clang++ -std=c++11 -Wall -g -c -I/opt/local/include -I/opt/local/include/eigen3 path_planner.cpp -o path_planner.o

# Linking
clang++ -L/opt/local/lib -lompl -lompl_app -lompl_app_base path_planner.o -o path_planner