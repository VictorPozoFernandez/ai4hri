import sqlite3

def create_tables(conn):
    cursor = conn.cursor()

    cursor.execute("""
    CREATE TABLE IF NOT EXISTS Camera (
        Product_ID INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
        Model VARCHAR(30) NOT NULL,
        Price VARCHAR(30) NOT NULL,
        Weight VARCHAR(30) NOT NULL,
        Type_of_camera VARCHAR(30) NOT NULL,
        Resolution VARCHAR(30) NOT NULL,
        ISO VARCHAR(30) NOT NULL)
    """)

    cursor.execute("""
    CREATE TABLE IF NOT EXISTS Camera_color (
        Product_ID INTEGER NOT NULL,
        Color VARCHAR(30) NOT NULL,
        PRIMARY KEY(Product_ID, Color),
        FOREIGN KEY(Product_ID) REFERENCES Camera(Product_ID) ON UPDATE CASCADE ON DELETE CASCADE)
    """)

    cursor.execute("""
    CREATE TABLE IF NOT EXISTS Camera_features (
        Product_ID INTEGER NOT NULL,
        Camera_features VARCHAR(50) NOT NULL,
        PRIMARY KEY(Product_ID, Camera_features),
        FOREIGN KEY(Product_ID) REFERENCES Camera(Product_ID) ON UPDATE CASCADE ON DELETE CASCADE)
    """)

    conn.commit()


def insert_camera(conn, model, price, weight, type_of_camera, resolution, iso):
    cursor = conn.cursor()
    cursor.execute("""
    INSERT INTO Camera (Model, Price, Weight, Type_of_camera, Resolution, ISO)
    VALUES (?, ?, ?, ?, ?, ?)
    """, (model, price, weight, type_of_camera, resolution, iso))

    conn.commit()
    return cursor.lastrowid


def insert_camera_color(conn, product_id, color):
    cursor = conn.cursor()
    cursor.execute("""
    INSERT INTO Camera_color (Product_ID, Color)
    VALUES (?, ?)
    """, (product_id, color))

    conn.commit()


def insert_camera_features(conn, product_id, camera_features):
    cursor = conn.cursor()
    cursor.execute("""
    INSERT INTO Camera_features (Product_ID, Camera_features)
    VALUES (?, ?)
    """, (product_id, camera_features))

    conn.commit()

def fetch_all_cameras(conn):
    cursor = conn.cursor()
    cursor.execute("SELECT * FROM Camera")
    return cursor.fetchall()


if __name__ == "__main__":
    conn = sqlite3.connect("/home/victor/catkin_ws/src/ai4hri/scripts/Camera_Store.db")
    
    #create_tables(conn)

    #product_id = insert_camera(conn, "Canon EOS 5D Mark III", "2000", "950 grams", "DSLR", "22.3 megapixels", "6400 with little noise")
    #insert_camera_color(conn, product_id, "black")
    #insert_camera_features(conn, product_id, "Silent Shooting")

    print("All cameras:")
    cameras = fetch_all_cameras(conn)
    for camera in cameras:
        print(camera)


    conn.close()