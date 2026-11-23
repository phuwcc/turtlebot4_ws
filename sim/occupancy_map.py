"""Read a ROS trinary occupancy map with the grid origin at the bottom left."""


def load_occupancy_map(yaml_path):
    import yaml
    from PIL import Image

    config = yaml.safe_load(yaml_path.read_text())
    if config.get('mode', 'trinary') != 'trinary':
        raise ValueError('Simulation requires a trinary occupancy map')
    with Image.open(yaml_path.parent / config['image']) as image:
        pixels = image.convert('L').transpose(getattr(Image, 'Transpose', Image).FLIP_TOP_BOTTOM)
    data = []
    for pixel in pixels.getdata():
        probability = (pixel if config['negate'] else 255 - pixel) / 255
        data.append(100 if probability > config['occupied_thresh']
                    else 0 if probability < config['free_thresh'] else -1)
    return dict(width=pixels.width, height=pixels.height, data=data,
                resolution=config['resolution'], origin=config['origin'])
