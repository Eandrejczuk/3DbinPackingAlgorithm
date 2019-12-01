from parcel import Parcel
from container import Container
from unusedSpace import defineLP, solveLP

if __name__ == '__main__':

    parcel1 = Parcel(0,5,5,5,15,1,1,1)
    parcel2 = Parcel(1,5,5,5, 8,1,1,1)
    parcel3 = Parcel(2,5,5,5, 15,1,1,1)
    parcel4 = Parcel(3,5,5,5, 20,1,1,1)
    parcel5 = Parcel(4,5,5,5,25,1,1,1)
    parcel6 = Parcel(5,5,5,5,3,1,1,1)
    parcel7 = Parcel(6,5,5,5, 25,1,1,1)
    parcel8 = Parcel(7,5,5,5, 30,1,1,1)
    parcel9 = Parcel(8,5,5,5, 30,1,1,1)
    parcel10 = Parcel(9,5,5,5,15,1,1,1)
    parcel11 = Parcel(10,5,5,5, 30,1,1,1)
    parcel12 = Parcel(11,5,5,5, 8,1,1,1)
    parcel13 = Parcel(12,5,5,5, 15,1,1,1)
    parcel14 = Parcel(13,5,5,5, 20,1,1,1)
    parcel15 = Parcel(14,5,5,5,25,1,1,1)
    parcel16 = Parcel(15,5,5,5,3,1,1,1)
    parcel17 = Parcel(16,5,5,5, 25,1,1,1)
    parcel18 = Parcel(17,5,5,5, 30,1,1,1)
    parcel19 = Parcel(18,5,5,5, 25,1,1,1)
    parcel20 = Parcel(19,5,5,5, 30,1,1,1)
    parcel21 = Parcel(20, 5, 5, 5, 15, 1, 1, 1)
    #parcel22 = Parcel(21, 5, 5, 5, 8, 1, 1, 1)
    #parcel23 = Parcel(22, 5, 5, 5, 15, 1, 1, 1)
    #parcel24 = Parcel(23, 5, 5, 5, 20, 1, 1, 1)
    #parcel25 = Parcel(24, 5, 5, 5, 25, 1, 1, 1)
    #print(Parcel.parcel_list)

    container = Container(0,20,20,20,3000)

    #print(Container.container_list)

    #for i in Parcel.parcel_list:
    #    print(i.calculate_volume())

    filename = str(len(Parcel.parcel_list)) + 'parcels'


    model = defineLP(Parcel.parcel_list, container, filename + '.lp',filename + '.sol', filename + 'coordinates.txt', 3600)
    solveLP(filename + '.lp', filename + '.sol', 3600)