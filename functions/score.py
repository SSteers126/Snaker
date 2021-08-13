def clamp(x, min, max):
    if x > max:
        return max
    elif x < min:
        return min
    else:
        return x

def smootherstep(edge0, edge1, x):
    x = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0)
    return x * x * (3 - 2 * x)