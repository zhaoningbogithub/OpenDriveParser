def get_string(node, name):
    return node.get(name)


def get_int(node, name):
    return int(node.get(name))


def get_float(node, name):
    if node.get(name) is not None:
        return float(node.get(name))
    else:
        return None