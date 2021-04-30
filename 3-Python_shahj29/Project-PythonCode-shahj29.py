import math                                     ## Python 2.8.9 and include Numpy, open3d and serial 
import numpy as np                              ## Jil Shah    400252316
import open3d as o3d
import serial

def txt_to_array(file_path):
    shortList =[4]
    RealValues = []
    i=0
    j=0 
    with (file_path) as file:      
        for line in file:
            i=0
            if not line.strip():    ## line stripped if its empty
                continue            ## move to the next line 
            for word in line.split(", "):    ##split each line with commas
                shortList.insert(i,word)     ## insert the words into the shorter array
                i+=1
            RealValues.insert(j,shortList[1])  ## only insert the 1st element into the realValues list 
            j+=1
        return RealValues;  ## the list realValues is returned
    

serial_port = 'COM7';    
baud_rate = 115200; 

total_angles = 24 
total_planes = 10
angle_changes = 15

output_file = open("output1.txt", "w+");
s = serial.Serial(serial_port, baud_rate)
line_numbers = 0
i=0
while (i<=9+total_angles*total_planes):   #repeat until all relevant lines are copied into a text file
    line = s.readline();
    line = line.decode("utf-8")
    print(line);
    if (i>(9)):  # ignore the initial 9 lines 
        output_file.write(line)
    i+=1;
    
output_file.close()   ## close the file




filepath = open("output1.txt",'r')    ## open the text file 
contents = txt_to_array(filepath)     ## converrt the text file into an array by calling the function
filepath.close()                      ## close the file

points =[]
angles = [0]
a=1
for a in range (total_angles-1):
    angles.append(angle_changes*a)    ## create an array of angles 0,15,30,45,...,345
    
i=0
plane = 200 
if __name__ == "__main__":
    file = open ("tof_radar.xyz","w")
    points=[]
    aI = 0 #angle counter
    for i in range (total_angles*total_planes):      ## go through all the data sent from the TOF sensor 
        rad = math.radians(angles[aI])               ## convert angle to radians and calculate y and z and add 200mm each time 
        x = plane
        y = float(contents[i]) * math.sin(rad)
        z = float(contents[i]) * math.cos(rad)
        points.append([x,y,z])
        aI = aI+1
        if (aI > total_angles-1):                    ## restart the angles after each plane
            aI=0
            plane+=200
        

    with file as f:
        file.writelines('{0} {1} {2}\n'.format(*xyz) for xyz in points)    ##write the x,y,z coordinates in the data file
    print ("Done")
    

    pcd = o3d.io.read_point_cloud("tof_radar.xyz", format = 'xyz')  ## create a point cloud with the x y and z points

    print(pcd)

    print (np.asarray(pcd.points))


    visual = o3d.visualization.Visualizer()  ##open the visualization
    visual.create_window()
    ##visual.add_geometry(o3d.geometry.TriangleMesh().create_coordinate_frame(size=300.0))
    lines = []
    
    pt1 = 0   ## set all points
    pt2 = 1
    pt3 = 2
    pt4 = 3
    pt5 = 4
    pt6 = 5
    pt7 = 6
    pt8 = 7
    pt9 = 8
    pt10 = 9
    pt11 = 10
    pt12 = 11
    pt13 = 12
    pt14 = 13
    pt15 = 14
    pt16 = 15
    pt17 = 16
    pt18 = 17
    pt19 = 18
    pt20 = 19
    pt21 = 20
    pt22 = 21
    pt23 = 22
    pt24 = 23
    
    po = 0  ##po is set to zero and changes each time by 24 which is the number of points per plane
    
    for x in range (total_planes):
        lines.append ([pt1+po,pt2+po])
        lines.append ([pt2+po,pt3+po])
        lines.append ([pt3+po,pt4+po])
        lines.append ([pt4+po,pt5+po])
        lines.append ([pt5+po,pt6+po])
        lines.append ([pt6+po,pt7+po])
        lines.append ([pt7+po,pt8+po])
        lines.append ([pt8+po,pt9+po])
        lines.append ([pt9+po,pt10+po])
        lines.append ([pt10+po,pt11+po])
        lines.append ([pt11+po,pt12+po])
        lines.append ([pt12+po,pt13+po])
        lines.append ([pt13+po,pt14+po])
        lines.append ([pt14+po,pt15+po])
        lines.append ([pt15+po,pt16+po])
        lines.append ([pt16+po,pt17+po])
        lines.append ([pt17+po,pt18+po])
        lines.append ([pt18+po,pt19+po])
        lines.append ([pt19+po,pt20+po])
        lines.append ([pt20+po,pt21+po])
        lines.append ([pt21+po,pt22+po])
        lines.append ([pt22+po,pt23+po])
        lines.append ([pt23+po,pt24+po])
        lines.append ([pt24+po,pt1+po])
        po+=24

    

    pt1 = 0
    pt2 = 1
    pt3 = 2
    pt4 = 3
    pt5 = 4
    pt6 = 5
    pt7 = 6
    pt8 = 7
    pt9 = 8
    pt10 = 9
    pt11 = 10
    pt12 = 11
    pt13 = 12
    pt14 = 13
    pt15 = 14
    pt16 = 15
    pt17 = 16
    pt18 = 17
    pt19 = 18
    pt20 = 19
    pt21 = 20
    pt22 = 21
    pt23 = 22
    pt24 = 23
    
    po = 0
    do = 24   ## do remains 24 
    
    for x in range (total_planes-1):
        lines.append ([pt1+po,pt1+do+po])   ## connect all planes with each other 
        lines.append ([pt2+po,pt2+do+po])
        lines.append ([pt3+po,pt3+do+po]) 
        lines.append ([pt4+po,pt4+do+po])
        lines.append ([pt5+po,pt5+do+po])
        lines.append ([pt6+po,pt6+do+po])
        lines.append ([pt7+po,pt7+do+po])
        lines.append ([pt8+po,pt8+do+po])
        lines.append ([pt9+po,pt9+do+po])
        lines.append ([pt10+po,pt10+do+po])
        lines.append ([pt11+po,pt11+do+po])
        lines.append ([pt12+po,pt12+do+po])
        lines.append ([pt13+po,pt13+do+po])
        lines.append ([pt14+po,pt14+do+po])
        lines.append ([pt15+po,pt15+do+po])
        lines.append ([pt16+po,pt16+do+po])
        lines.append ([pt17+po,pt17+do+po])
        lines.append ([pt18+po,pt18+do+po])
        lines.append ([pt19+po,pt19+do+po])
        lines.append ([pt20+po,pt20+do+po])
        lines.append ([pt21+po,pt21+do+po])
        lines.append ([pt22+po,pt22+do+po])
        lines.append ([pt23+po,pt23+do+po])
        lines.append ([pt24+po,pt24+do+po])
        po+=24  ## increase po by 24 for each phase 


    line_set = o3d.geometry.LineSet(points = o3d.utility.Vector3dVector(np.asarray(pcd.points)),
                                               lines = o3d.utility.Vector2iVector(lines))
    
    visual.remove_geometry(line_set)
    line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),
                                    lines=o3d.utility.Vector2iVector(lines))
    visual.add_geometry(line_set)
    visual.poll_events()
    visual.update_renderer()

    


    visual.run()   ##visualize the model in open3D



file.close()
    
