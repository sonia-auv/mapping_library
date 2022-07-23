hydro =[0.150, 0, 0.118]

q = quatinv(eul2quat(deg2rad([180,0,90]),"ZYX"))

quatrotate(q,hydro)
