import argparse
from .main_sim import simulation, visualisation
import matplotlib.pyplot as plt
import numpy as np


def main():
    # Argument parser
    parser = argparse.ArgumentParser(
        description="Simulate flight performances"
    )

    parser.add_argument("plotfile", type=str,
                        help="Path of input .txt file")


    args = parser.parse_args()
    """
    Input arguments sanity check
    """
    if args.plotfile is None:
        raise ValueError("Must specify input_file.txt") #Mettre aussi input validation pour la forme du input file
    
    # Getting input values from user
    with open(args.plotfile, 'r') as file:
        input_lines = [line.strip() for line in file]

    inputs = {}

    for l in input_lines:
        if "#" in l:
            l = l[:(l.find("#"))]
        if len(l) != 0:
            line = l.split("=")
            inputs[line[0].strip()] = line[1].strip()

    #print(inputs)

    raise NotImplementedError

    #TODO
    """
    # Running simulation
    sim = simulation(0.004, 4, "some_mode")

    # Prepare data
    visualisation(sim)
    """



if __name__ == "__main__":
    main()
