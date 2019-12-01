class Container(object):
    container_list=[]
    def __init__(self, id, length, width, height, max_weight):
        self.id = id
        self.length = length
        self.width = width
        self.height = height
        self.weight = max_weight
        self.container_list.append(self)

# methods
    def calculate_volume(self):
        return self.length*self.width*self.height

# to_string
    def __str__(self):
            return ', '.join([
                "id: " + str(self.id),
                "length: " + str(self.length),
                "width: " + str(self.width),
                "height: " + str(self.width),
                "weight: " + str(self.weight)
            ])
    def __repr__(self):
        return self.__str__()