def convert_box_to_yolo(x, y, w, h, img_width, img_height):
    """
    Converts standard bounding box [x, y, w, h] (top-left) 
    to YOLO format [x_center, y_center, width_norm, height_norm].
    """
    x_c = (x + w / 2) / img_width
    y_c = (y + h / 2) / img_height
    w_n = w / img_width
    h_n = h / img_height
    
    return x_c, y_c, w_n, h_n