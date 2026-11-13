from PIL import Image
import pandas as pd
import math
import os

class Picturemaker:

    def makeAPic(save_name, width = 128, height = 128):
        middle_x = width / 2
        middle_y = height / 2

        max_radius = min(width, height) / 2

        # Create a new Image with correct background colour
        image = Image.new("RGB", (width, height), (61, 127, 139))
        pixel = image.load()

        # Decide colour for each pixel
        for y in range(height):
            for x in range(width):

                dx = x - middle_x
                dy = y - middle_y
                radius = math.sqrt(dx**2 + dy**2)

                r = radius / max_radius

                if r>1:
                    continue

                ring = int(r * 5)

                if ring >= 5:
                    ring = 4

                pixel[x, y] = colours[ring]
        
        image.save(f'{cd_images}{save_name}.png')
        print(f"Image successfully saved.")

    def createImages(amount):
        for _ in range(amount):
            i=0
            while(os.path.exists(f'{cd_images}{i}.png')):
                i+=1

            Picturemaker.makeAPic(i)

            s = "".join(colour_dict[a] for a in colours)

            if os.path.exists(cd_data):
                pandas_farben = pd.read_csv(cd_data, header=None, index_col=0)
                pandas_farben.loc[len(pandas_farben)] = s
            else:
                pandas_farben = pd.DataFrame({1: [s]})
                pandas_farben.index = [0]

            pandas_farben.to_csv(cd_data, header=False)

    def shuffleColours():
        r = range(5)
        for v in r:
            for w in r:
                for x in r:
                    for y in r:
                        for z in r:
                            colours[0]=shuffle_colours_dict[z]
                            colours[1]=shuffle_colours_dict[y]
                            colours[2]=shuffle_colours_dict[x]
                            colours[3]=shuffle_colours_dict[w]
                            colours[4]=shuffle_colours_dict[v]
                            Picturemaker.createImages(1)
            

cd_images = "G:\\Users\\Jonas Möhlenpage\\Documents\\Schule\\Robotik\\Toller Ordner\\Bilder\\"
cd_data = "G:\\Users\\Jonas Möhlenpage\\Documents\\Schule\\Robotik\\Toller Ordner\\daten.csv"

colour_dict = {
    (0, 207, 0):"g",
    (0, 0, 207):"b",
    (207, 0, 0):"r",
    (0, 0, 0):"s",
    (207, 207, 0):"y"
}

red = (207, 0, 0)
green = (0, 207, 0)
blue = (0, 0, 207)
black = (0, 0, 0)
yellow = (207, 207, 0)
colours = [red, green, blue, black, yellow]

shuffle_colours_dict  = {
    0:red,
    1:green,
    2:blue,
    3:black,
    4:yellow
}

