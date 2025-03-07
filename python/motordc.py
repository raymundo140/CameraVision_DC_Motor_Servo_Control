import cv2
import numpy as np
import serial
import time

# Configurar el puerto Serial y la velocidad 
ser = serial.Serial('COM5', 115200)
time.sleep(2)  

# Iniciar captura de la cámara
cap = cv2.VideoCapture(0)

def detectar_color(frame):
    """
    Convierte la imagen a HSV y crea máscaras para rojo, verde y amarillo.
    Retorna el color predominante y las máscaras para visualización.
    Si ninguno de los colores tiene una cantidad significativa de píxeles,
    se retorna "NONE".
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Máscara para ROJO
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    
    # Máscara para VERDE 
    lower_green = np.array([40, 70, 70])
    upper_green = np.array([80, 255, 255])
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    
    # Máscara para AMARILLO
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    # Contar píxeles no nulos en cada máscara
    count_red = cv2.countNonZero(mask_red)
    count_green = cv2.countNonZero(mask_green)
    count_yellow = cv2.countNonZero(mask_yellow)
    
    # Definir un umbral para considerar que hay detección de color
    threshold = 100  # Ajusta este valor según la resolución y condiciones de iluminación
    
    # Si el máximo de píxeles detectados es menor al umbral, consideramos que no se detectó ninguno
    max_count = max(count_red, count_green, count_yellow)
    if max_count < threshold:
        return "NONE", mask_red, mask_green, mask_yellow

    # Seleccionar el color con mayor cantidad de píxeles
    colores = {"RED": count_red, "GREEN": count_green, "YELLOW": count_yellow}
    color_predominante = max(colores, key=colores.get)
    
    return color_predominante, mask_red, mask_green, mask_yellow

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Detectar el color predominante en la imagen
    color, mask_red, mask_green, mask_yellow = detectar_color(frame)
    
    # Asignar velocidad según el color detectado
    if color == "YELLOW":
        speed = 180
    elif color == "GREEN":
        speed = 210
    elif color == "RED":
        speed = 255
    else:  # Si no se detecta ninguno de los colores 
        speed = 0
    
    # Enviar el valor por Serial, finalizando con salto de línea
    mensaje = f"{speed}\n"
    ser.write(mensaje.encode('utf-8'))
    
    # Mostrar en la imagen el color detectado y la velocidad asignada
    cv2.putText(frame, f"Color: {color} - Speed: {speed}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    # Mostrar la imagen original y las máscaras para depuración
    cv2.imshow("Frame", frame)
    cv2.imshow("Mascara Rojo", mask_red)
    cv2.imshow("Mascara Verde", mask_green)
    cv2.imshow("Mascara Amarillo", mask_yellow)
    
    # Presionar ESC para salir
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
ser.close()
