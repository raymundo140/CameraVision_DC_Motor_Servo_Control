import cv2
import numpy as np
import serial
import time

# 1. Configurar el puerto Serial y velocidad
ser = serial.Serial('COM5', 115200)  # Ajusta 'COM5' o el puerto que uses
time.sleep(2)  # Espera un par de segundos para que el puerto se inicie

# 2. Iniciar captura de la cámara
cap = cv2.VideoCapture(0)  # 0 para la webcam por defecto

def procesar_imagen(frame):
    """
    Filtra la imagen para que se vean solo las áreas rojas utilizando el canal Hue.
    Luego, sobre la máscara roja se aplican Canny y HoughLinesP para detectar la primera línea
    y calcular su ángulo en grados.
    Retorna el ángulo calculado y la máscara roja para visualización.
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
    
    # Cálculo del ángulo
    angle_degrees = 90  # valor por defecto si no hay líneas
    if lines is not None:
        x1, y1, x2, y2 = lines[0][0]
        angle_radians = np.arctan2((y2 - y1), (x2 - x1))
        angle_degrees = np.degrees(angle_radians)
        angle_degrees = angle_degrees % 180

    return int(angle_degrees), red_mask

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 3. Procesar imagen: obtener ángulo y máscara
    angle, red_mask = procesar_imagen(frame)

    # 4. Enviar el ángulo por Serial (terminado con salto de línea)
    mensaje = f"{angle}\n"
    ser.write(mensaje.encode('utf-8'))

    # 5. Mostrar resultados para depuración
    cv2.putText(frame, f"Angulo: {angle}", (10,30), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
    cv2.imshow("Frame", frame)
    
    # Ahora sí podemos mostrar la máscara roja y los bordes, 
    # pues tenemos 'red_mask' en esta parte del código
    cv2.imshow("Mascara Roja", red_mask)
    cv2.imshow("Bordes", cv2.Canny(red_mask, 50, 150))

    # Presiona ESC para salir
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
ser.close()
