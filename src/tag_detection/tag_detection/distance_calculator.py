#!/usr/bin/env python3
import rclpy
import tf2_ros
import numpy as np  # Für effiziente Vektorberechnung
import pandas as pd # Für die Datenspeicherung
from geometry_msgs.msg import TransformStamped
import os

# Definiere den Speicherpfad (ersetze 'harry' durch deinen Usernamen)
save_path = os.path.expanduser('~/robot/tag_detections_final.csv')


class TagListenerNode:
    def __init__(self):
        self.nh = rclpy.create_node('tf2_listener_advanced')
        
        # TF Setup
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self.nh)
        
        # Pandas Setup: Erstellt eine leere Tabelle mit Spaltenköpfen
        self.data_log = pd.DataFrame(columns=['timestamp', 'tag_id', 'dist', 'x', 'y', 'z'])
        
        # Timer (0.25s)
        self.nh.create_timer(0.25, self.timercallback)
        self.nh.get_logger().info("Advanced Listener mit NumPy & Pandas gestartet.")

    def timercallback(self):
        from_frame = "base_link"
        target_tags = ["tag16h5:1", "tag16h5:2", "tag16h5:7"] 
        
        for to_frame in target_tags:
            try:
                trans = self.tfBuffer.lookup_transform(
                    from_frame, to_frame, rclpy.time.Time(), 
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                
                # 1. NumPy für die Position nutzen
                pos = np.array([
                    trans.transform.translation.x,
                    trans.transform.translation.y,
                    trans.transform.translation.z
                ])
                
                # Euklidische Distanz mit NumPy (Länge des Vektors)
                dist = np.linalg.norm(pos)
                
                # 2. Daten in Pandas Dataframe speichern
                new_entry = {
                    'timestamp': self.nh.get_clock().now().to_msg().sec,
                    'tag_id': to_frame,
                    'dist': dist,
                    'x': pos[0],
                    'y': pos[1],
                    'z': pos[2]
                }
                
                # Eintrag hinzufügen
                self.data_log = pd.concat([self.data_log, pd.DataFrame([new_entry])], ignore_index=True)
                
                # Ausgabe im Terminal
                self.nh.get_logger().info(f"Tag {to_frame} erkannt! Distanz: {dist:.2f}m")
                
                # Optional: Alle 10 Messungen in eine CSV speichern
                if len(self.data_log) % 10 == 0:
                    self.data_log.to_csv('tag_detections.csv', index=False)
                    self.nh.get_logger().info("Daten-Log in 'tag_detections.csv' aktualisiert.")

            except Exception:
                pass

def main():
    rclpy.init()
    node = TagListenerNode()
    try:
        rclpy.spin(node.nh)
    except KeyboardInterrupt:
        # Beim Beenden letzte Daten speichern
        node.data_log.to_csv('tag_detections_final.csv', index=False)
        node.nh.destroy_node()
        rclpy.shutdown()
        # Ändere den Speicherbefehl am Ende (main-Funktion):
        node.data_log.to_csv(save_path, index=False)
        print(f"Daten erfolgreich gespeichert unter: {save_path}")

if __name__ == '__main__':
    main()