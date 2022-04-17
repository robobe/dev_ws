import math

SEA_LEVEL_PRESSURE = 1013.25

def hypsometric_formula(pa:float, t:float) -> float:
    p_ratio = (SEA_LEVEL_PRESSURE/pa)
    h = ((math.pow(p_ratio, 1/5.257)-1) * (t+273.15)) / 0.0065
    return h

if __name__ == "__main__":
    print(hypsometric_formula(70, 15))