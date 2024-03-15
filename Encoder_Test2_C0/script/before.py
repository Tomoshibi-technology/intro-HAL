#!/usr/bin/python3
import os

os.chdir(os.path.dirname(os.path.abspath(__file__)))
if os.path.exists("./../Core/Src/main.cpp"):
	os.rename("./../Core/Src/main.cpp", "./../Core/Src/main.c")