path1 = []
path2 = []
path3 = []

def lectura():

    with open('taskfile_tampconfig_chess_1_simple.xml', 'r') as file:
        while True:
            line = file.readline()
        
            if line.strip() == '<Transit>' and path1 == []:
                line = file.readline()
                while line.strip() != '</Transit>': 
                    conf = line.strip().strip('<Conf>').strip('</Conf>').strip().split(' ')
                    conf = [float(num) for num in conf]
                    path1.append(conf)
                    line = file.readline()
            elif '<Transfer' in line.strip() and path2 == []:
                line = file.readline()
                while line.strip() != '</Transfer>':
                    conf = line.strip().strip('<Conf>').strip('</Conf>').strip().split(' ')
                    conf = [float(num) for num in conf]
                    path2.append(conf)
                    line = file.readline()
            elif line.strip() == '<Transit>' and path3 == []:
                line = file.readline()
                while line.strip() != '</Transit>':
                    conf = line.strip().strip('<Conf>').strip('</Conf>').strip().split(' ')
                    conf = [float(num) for num in conf]
                    path3.append(conf)
                    line = file.readline()


        

            if not line:
                break
        

# El fichero se cierra automáticamente al salir del bloque with

lectura()
