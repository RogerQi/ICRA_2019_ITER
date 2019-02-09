#!/usr/bin/env python3
import sys
sys.path.append("../.")
import simulation.robot

def main():
    my_first_robot = simulation.robot.robot()
    print(my_first_robot._map)

if __name__ == '__main__':
    main()
