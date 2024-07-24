import pyrealsense2 as rs
import numpy as np
import cv2

# Configurar o pipeline
pipeline = rs.pipeline()
config = rs.config()

# Configurar a câmera para capturar em 640x480 a 30 FPS
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Iniciar o fluxo
pipeline.start(config)

try:
    while True:
        # Capturar um frame
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            continue

        # Converter a imagem de profundidade para um array numpy
        depth_image = np.asanyarray(depth_frame.get_data())

        # Aplicar coloração para melhor visualização
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Exibir a imagem
        cv2.imshow('RealSense', depth_colormap)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Parar o pipeline
    pipeline.stop()
    cv2.destroyAllWindows()
