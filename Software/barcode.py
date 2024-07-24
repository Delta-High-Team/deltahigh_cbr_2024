import numpy as np
import cv2
import time
import math
from threading import Thread
from pyzbar import pyzbar

# Define a codec e cria o objeto VideoWriter
fourcc = cv2.VideoWriter_fourcc(*'XVID')
frame_size = (640, 480)
out = cv2.VideoWriter('barcode.avi', fourcc, 10.0, frame_size)

def decode(frame, preprocessed_frame):
    # Decodifica QR codes e códigos de barras na imagem
    decoded_objects = pyzbar.decode(preprocessed_frame)
    
    for obj in decoded_objects:
        # Desenha o retângulo em torno do código
        (x, y, w, h) = obj.rect
        cv2.rectangle(preprocessed_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # Coloca o texto do código na imagem
        text = f'{obj.type}: {obj.data.decode("utf-8")}'
        cv2.putText(preprocessed_frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        print(obj.data.decode("utf-8"))
    
    return frame, decoded_objects

def preprocess(frame):
    return frame

try:
    cap = cv2.VideoCapture(0)

    # Inicia variavel para medir FPS
    prevFrame = 0

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Calula o FPS atual
        actualFrame = time.time()
        frame_rate = round(1.0/(actualFrame-prevFrame))
        prevFrame = actualFrame    
        # print(frame_rate)

        # Aplica pré-processamento no frame
        preprocessed_frame = preprocess(frame)

        # Processa o frame para detectar QR codes e códigos de barras
        frame, decoded_objects = decode(frame, preprocessed_frame)

        # Exibir o FPS
        cv2.putText(frame, f"FPS: {frame_rate}", (frame_size[0]-120, frame_size[1]-30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2, cv2.LINE_AA)
        
        # Escreve o frame processado no arquivo de vídeo
        # out.write(frame)
        # cv2.imshow('Camera', frame)

        # Sai do loop ao pressionar 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    # Libera a captura e o objeto de gravação e fecha todas as janelas
    cap.release()
    cv2.destroyAllWindows()