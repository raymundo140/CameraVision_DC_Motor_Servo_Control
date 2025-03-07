import cv2
import numpy as np
import serial
import time

# 1. Configurar el puerto Serial y velocidad
ser = serial.Serial('COM5', 115200)  # Ajusta 'COM5' o el puerto que uses
time.sleep(2)  # Espera un par de segundos para que el puerto se inicie

# 2. Iniciar captura de la cámara
cap = cv2.VideoCapture(0)  # 0 para la webcam por defecto

def obtener_angulo(frame):
    """
    Filtra la imagen para que se vean solo las áreas rojas utilizando el canal Hue.
    Luego, sobre la máscara roja se aplican Canny y HoughLinesP para detectar la primera línea
    y calcular su ángulo en grados.
    Si no se detecta ninguna línea, retorna 90 como valor por defecto.
    """
    # Convertir a HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Definir rangos para el color rojo en HSV (dos rangos por la naturaleza circular del Hue)
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    
    # Crear máscaras para cada rango y combinarlas
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(mask1, mask2)
    
    # Opcional: aplicar una operación morfológica para reducir ruido
    kernel = np.ones((5, 5), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
    
    # Detectar bordes en la máscara roja
    edges = cv2.Canny(red_mask, 50, 150)
    
    # Detectar líneas con HoughLinesP
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, minLineLength=50, maxLineGap=10)
    
    if lines is not None:
        # Tomamos la primera línea detectada
        x1, y1, x2, y2 = lines[0][0]
        
        # Calcular el ángulo en radianes y luego convertir a grados
        angle_radians = np.arctan2((y2 - y1), (x2 - x1))
        angle_degrees = np.degrees(angle_radians)
        
        # Ajustar para que esté en el rango [0, 180]
        angle_degrees = angle_degrees % 180
        
        return int(angle_degrees)
    
    # Si no se detecta línea, retorna 90 como valor neutro
    return 90

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 3. Obtener ángulo de la línea (trabajando sobre la máscara roja)
    angle = obtener_angulo(frame)

    # 4. Enviar el ángulo por Serial (terminado con salto de línea)
    mensaje = f"{angle}\n"
    ser.write(mensaje.encode('utf-8'))

    # 5. Mostrar resultados para depuración:
    # Mostrar la máscara roja y el frame original con el ángulo dibujado
    cv2.putText(frame, f"Angulo: {angle}", (10,30), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
    cv2.imshow("Frame", frame)
    
    # Opcional: mostrar la máscara roja y los bordes detectados
    cv2.imshow("Mascara Roja", red_mask)
    cv2.imshow("Bordes", cv2.Canny(red_mask, 50, 150))

    # Presiona ESC para salir
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
ser.close()
