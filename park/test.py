import numpy as np
from sklearn.linear_model import LinearRegression
import matplotlib.pyplot as plt

# Supongamos que "data" es tu conjunto de datos lidar con columnas [x, y]

inf = float('inf')

data=[inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,inf ,3.2 ,3.1 ,3.0 ,2.8 ,2.7 ,2.6 ,2.5 ,2.5 ,2.5 ,2.4 ,2.4 ,2.3 ,2.3 ,2.3 ,2.3 ,2.2 ,2.2 ,2.2 ,2.1 ,2.1 ,2.1 ,2.1 ,2.1 ,2.1 ,2.0 ,2.0 ,2.0 ,2.0 ,2.0 ,2.0 ,1.9 ,1.9 ,1.9 ,1.9 ,1.9 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.9 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.7 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.8 ,1.9 ,1.9 ,1.9 ,1.9 ,1.9 ,1.9 ,1.9 ,1.9 ,1.9 ,2.0 ,1.9 ,1.9 ,1.9 ,2.0 ,2.0 ,2.0 ,2.1 ,2.1 ,2.1 ,2.1 ,2.2 ,2.2 ,2.2 ,2.3 ,2.3 ,2.4 ,2.4 ,2.5 ,2.5 ,2.6 ,2.6 ,inf ,inf ,inf ,4.6 ,4.6 ,4.4 ,4.4 ,4.3 ,4.2 ,4.2 ,4.1 ,4.1 ,4.0 ,4.0 ,4.0 ,4.0 ,4.0 ,3.9 ,3.9 ,3.9 ,3.9 ,4.0 ,4.1 ,4.3 ,4.3 ,4.4 ,4.7 ,5.3 ,5.7 ,6.1 ,6.7 ,7.2 ,10.0 ,10.0 ,10.1 ,10.4 ,12.8 ,21.6 ,21.6 ,21.7 ,27.6 ,inf ,inf]

# convertir datos polares a cartesianos
def polar_to_cartesian(r, theta):
    return r * np.cos(theta), r * np.sin(theta)

def calculate_main_direction(lidar_data):
    # Agregar una columna de unos para el término independiente
    X = np.column_stack((np.ones_like(lidar_data[:, 0]), lidar_data[:, 0]))
    # Calcular los coeficientes de regresión usando la pseudo-inversa de X
    coefficients = np.linalg.pinv(X) @ lidar_data[:, 1]
    print('Coeficientes de regresión: {}'.format(coefficients))
    return coefficients

lidar_data = []
for i in range(len(data)):
    if(data[i]==inf):
        #data[i]=5
        continue
    if(i<0 or i>180):
        continue
    #print(i/180*np.pi)
    lidar_data.append(polar_to_cartesian(data[i],i/180*np.pi))

#print(lidar_data)
lidar_data=np.array(lidar_data)


coefficients=calculate_main_direction(lidar_data)
# Visualiza los resultados
plt.scatter(lidar_data[:, 0], lidar_data[:, 1], label='Datos Lidar', color='blue', marker='.')
#generate values to draw the line
x_values = np.linspace(-5, 5, 100)
y_values = coefficients[0] + coefficients[1] * x_values
#plot the line
# Dibuja el punto (0, 0)
plt.scatter(0, 0, color='green', marker='o', label='Punto (0, 0)')
plt.plot(x_values, y_values, color='red', linewidth=2, label='Ajuste Lineal')

plt.xlim(left=-5, right=5)
plt.ylim(bottom=-1, top=7)
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.show()