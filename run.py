
import rover
import json
import sys
import warnings


if __name__ == "__main__":
    warnings.filterwarnings("ignore")
    if len(sys.argv) <= 1:
        rover.run()
    else:
        with open(sys.argv[1], "r") as f:
            config_dict = json.loads(f.read())
            rover.run(**config_dict)
