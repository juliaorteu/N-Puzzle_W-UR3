"""
#########################################
###### Extract join configurations ######
#########################################

This Python script reads an XML file ('taskfile_tampconfig_chess_1_simple.xml') and extracts joint configurations
from specified sections ('<Transit>' and '<Transfer>'). The configurations are stored in separate lists: path1, path2,
and path3. The script demonstrates parsing an XML-like file format and processing its contents.
"""
path1, path2, path3, path4, path5, path6 = [], [], [], [], [], []

def read_file():
    """
    Reads the XML file and extracts joint configurations from the '<Transit>' and '<Transfer>' sections.
    :returns: The configurations
    """
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
            elif line.strip() == '<Transit>' and path4 == []:
                line = file.readline()
                while line.strip() != '</Transit>':
                    conf = line.strip().strip('<Conf>').strip('</Conf>').strip().split(' ')
                    conf = [float(num) for num in conf]
                    path4.append(conf)
                    line = file.readline()
            
            elif line.strip() == '<Transfer>' and path5 == []:
                line = file.readline()
                while line.strip() != '</Transfer>':
                    conf = line.strip().strip('<Conf>').strip('</Conf>').strip().split(' ')
                    conf = [float(num) for num in conf]
                    path5.append(conf)
                    line = file.readline()
            elif line.strip() == '<Transit>' and path6 == []:
                line = file.readline()
                while line.strip() != '</Transit>':
                    conf = line.strip().strip('<Conf>').strip('</Conf>').strip().split(' ')
                    conf = [float(num) for num in conf]
                    path6.append(conf)
                    line = file.readline()
                    
            if not line:
                break
        
        
    return path1, path2, path3, path4, path5, path6

