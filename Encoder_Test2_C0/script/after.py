import os

os.chdir(os.path.dirname(os.path.abspath(__file__)))
os.rename("./../Core/Src/main.c", "./../Core/Src/main.cpp")