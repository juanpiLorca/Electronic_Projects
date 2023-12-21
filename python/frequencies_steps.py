# num of samples: 
N = 128 

# clock frequency: 
fc = 4000

# tones frequency: 
notes = []
frequencies = []
with open("frequencies.txt", "r") as file: 
    for line in file: 
        content = line.split(",")
        note = content[0]
        notes.append(note)
        f = float(content[1][:-2])
        frequencies.append(f)


# number of digits fractional part: 
ndigits = 4

steps = []
for f in frequencies: 
    step = round(f * (4*N) / fc, ndigits=ndigits)
    steps.append(float(step))

for s in steps: 
    print(f"frequency: {s}")
    print(f"int part: {int(s)}, fractional part: {round(s - int(s), ndigits=ndigits)}")


# Writing two files:
def int_frac(num, n=ndigits): 
    """
    Args: 
        - num: number to split 
        - n: order 
    """
    integer = int(num)
    fraction = int((num - int(num)) * 10**n)
    return integer, fraction


int_part_file = "integer_part.txt"
frac_part_file = "fractional_part.txt"
# amount of hex: 
h = 4
with open(int_part_file, "w") as file: 
    for step in steps:
        step_int, _ = int_frac(step)
        file.write(format(step_int, f"0{h}X")+"\n")

with open(frac_part_file, "w") as file: 
    for step in steps:
        _, step_frac = int_frac(step)
        file.write(format(step_frac, f"0{h}X")+"\n")

# Frequencies using 8 bit for each par: integer and fractional:
freqs_file = f"frequencies_{2*h}hex.txt"

coe_content = "memory_initialization_radix=16;\n"
coe_content += "memory_initialization_vector=\n"

with open(freqs_file, "w") as file: 
    file.write(coe_content)
    for i in range(0,len(steps)):
        step_int, step_frac = int_frac(steps[i])
        if i == len(steps) - 1:
            file.write(format(step_int, f"0{h}X")+format(step_frac, f"0{h}X")+";")
        else:
            file.write(format(step_int, f"0{h}X")+format(step_frac, f"0{h}X")+"\n")





