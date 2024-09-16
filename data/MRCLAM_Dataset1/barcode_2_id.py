### this translate the barcode to ID in the measurement files for MRCLAM_Dataset


### set up barcode dictionary

barcode_file = open("Barcodes.dat", 'r')

barcode_dict = {}

s = barcode_file.readline()

while(s):
	if(s[0]!='#'):
		barcode_dict.update({s.split( )[1]:s.split( )[0]}) #barcode: subject

	s = barcode_file.readline()

barcode_file.close()


###
for i in range(1,6):
    measure_file = open('Robot'+str(i)+'_Measurement.dat', 'r')
    measure_out_file = open('Robot'+str(i)+'_Measurement_x.dat', 'w')
    print i
    s = measure_file.readline()
    while(s):
        if(s[0]!='#'):
            key = s.split()[1]
            landmark_id = barcode_dict.get(key)
            if(landmark_id):
                out_s = s.split()[0] + ' ' + landmark_id + ' ' + s.split()[2] + ' ' + s.split()[3] + '\n'
                measure_out_file.write(out_s)
        s = measure_file.readline()

    measure_file.close()
    measure_out_file.close()
