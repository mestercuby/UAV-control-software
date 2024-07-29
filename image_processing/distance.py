import math

class distance:

    def hesap(self,height,pos,fov,camera_angle):
        mesafe= [0,0]
        x=height/math.cos(camera_angle/57.3)
        L=2*x*math.sin(fov[1]/114.6)
        m= (480-pos[1]) / 480 * L
        n= pos[1] / 480 * L
        h0 = x * math.cos(fov[1]/114.6)
        angle = math.asin(L/(2*x)) - math.atan((n-m)/(2*h0)) 
        mesafe[0]= height * math.tan(camera_angle/57.3 + angle)


        v = height / math.cos(camera_angle/57.3 + angle)
        p = v * math.tan(fov[0]/114.6)
        mesafe[1]=(320-pos[0])/320 * p

        return mesafe
    
"""
    def get_point_at_distance(self, R=6371):
        
        lat: initial latitude, in degrees
        lon: initial longitude, in degrees
        d: target distance from initial
        bearing: (true) heading in degrees
        R: optional radius of sphere, defaults to mean radius of earth

        Returns new lat/lon coordinate {d}km from initial, in degrees
      
        d=self.hesap(10,[100,100],[100,100],100)
        lat1 = radians(self.lat)
        lon1 = radians(self.lon)
        a = radians(self.heading)
        lat2 = asin(sin(lat1) * cos(d / R) + cos(lat1) * sin(d / R) * cos(a))
        lon2 = lon1 + atan2(
            sin(a) * sin(d / R) * cos(lat1),
            cos(d / R) - sin(lat1) * sin(lat2)
        )
        return degrees(lat2), degrees(lon2)
          """


string=""
command=string.split(' ')[0]
if command=='ironi':
    argument=string.split(' ')[1].split(',')
    argument=[float(a) for a in argument]
print(command)