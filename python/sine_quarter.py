import numpy as np 
import matplotlib.pyplot as plt 

txtFile = "sine_samples.txt"

N = 128
samples = np.linspace(0, np.pi/2, N)
quarter_sine = (128 * np.sin(samples))

qsine_list = []
for s in quarter_sine: 
    qsine_list.append(s)

print(quarter_sine.shape)
plt.plot(quarter_sine)
plt.title("cuarto de onda sinusoidal")
plt.xlabel("muestras")
plt.ylabel("amplitud")
plt.grid()
plt.show()

coe_content = "memory_initialization_radix=16;\n"
coe_content += "memory_initialization_vector=\n"

with open(txtFile, "w") as file: 
    file.write(coe_content)
    for sample in qsine_list: 
        n = int(sample)
        file.write(format(n, "02X")+"\n")


