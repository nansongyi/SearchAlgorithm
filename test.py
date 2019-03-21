
def out_of_range(x):
    return x <= 99 and x >= 0

def check_range(p):
    return not all(map(out_of_range, p))

print(check_range([1,2]))
