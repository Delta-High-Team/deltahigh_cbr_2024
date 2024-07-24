###########################################################################################
#                   Hello World da camera Intel Realsense D435
###########################################################################################
# OBS:
#   - Obtem os dados RGB-D
#   - Exibe a imagem RGB e a imagem de profundidade (atraves de mapa de cores)
#   - Print da distancia do pixel no centro da imagem
#   - Alinha as duas imagens (os pixels nas duas imagens representam a mesma coisa)
###########################################################################################

import pyrealsense2 as rs
import numpy as np
import cv2

# Criar um pipeline
pipeline = rs.pipeline()
config = rs.config()

# Configurar o pipeline para habilitar os streams de profundidade e cor
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Iniciar o pipeline
pipeline.start(config)

# Criar um objeto para alinhar a profundidade ao stream de cor
align_to = rs.stream.color
align = rs.align(align_to)

try:
    while True:
        # Capturar um conjunto de frames
        frames = pipeline.wait_for_frames()
        
        # Alinhar os frames de profundidade ao frame de cor
        aligned_frames = align.process(frames)
        
        # Obter os frames alinhados de profundidade e cor
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        
        if not aligned_depth_frame or not color_frame:
            continue
        
        # Converter os frames para arrays numpy
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        # Aplicar coloração à imagem de profundidade
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Distância do pixel no centro da imagem
        center_x = color_image.shape[1] // 2
        center_y = color_image.shape[0] // 2
        cv2.circle(color_image, (center_x, center_y), 5, (0, 0, 255), -1)       # Desenhar um círculo vermelho sólido
        cv2.circle(depth_colormap, (center_x, center_y), 5, (0, 0, 255), -1)    # Desenhar um círculo vermelho sólido
        
        distance = aligned_depth_frame.get_distance(center_x, center_y)
        print('Distancia do pixel no centro da image = ' + str(distance))
        
        # Combinar imagens de cor e profundidade lado a lado
        images = np.hstack((color_image, depth_colormap))
        
        # Exibir a imagem combinada
        cv2.imshow('Aligned Images', images)
        
        # Verificar se a tecla 'q' foi pressionada
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Parar o pipeline
    pipeline.stop()
    cv2.destroyAllWindows()
