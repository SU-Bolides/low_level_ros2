import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time


class ESCCalibration(Node):
    def __init__(self):
        super().__init__('esc_calibration')
        
        # Publisher pour envoyer les commandes sur cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Float32, '/cmd_vel', 10)
        
        # Valeurs de vitesse pour la calibration
        self.VEL_MAX = 1.0       # Plein avant
        self.VEL_NEUTRAL = 0.0   # Point neutre
        self.VEL_MIN = -1.0      # Plein arrière
        
        self.get_logger().info('ESC Calibration Node initialisé')
        
    def send_velocity(self, linear_x):
        """Envoie une commande de vitesse à l'ESC via cmd_vel"""
        msg = Float32()
        msg.data = linear_x
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info(f'Envoi cmd_vel: {linear_x}')
        
    def calibrate(self):
        """Séquence de calibration de l'ESC Tamiya TBLE-04S"""
        self.get_logger().info('=== DÉBUT DE LA CALIBRATION ESC ===')
        self.get_logger().info('Assurez-vous que l\'ESC est allumé!')
        
        # Étape 1: Envoyer la valeur maximale
        self.get_logger().info(f'Étape 1: Envoi de la valeur MAX ({self.VEL_MAX})')
        self.send_velocity(self.VEL_MAX)
        input('Appuyez sur le bouton de l\'ESC pour enregistrer MAX, puis appuyez sur ENTRÉE...')
        
        # Étape 2: Envoyer la valeur minimale
        self.get_logger().info(f'Étape 2: Envoi de la valeur MIN ({self.VEL_MIN})')
        self.send_velocity(self.VEL_MIN)
        input('Appuyez sur le bouton de l\'ESC pour enregistrer MIN, puis appuyez sur ENTRÉE...')
        
        # Étape 3: Retour au neutre
        self.get_logger().info(f'Étape 3: Retour au NEUTRE ({self.VEL_NEUTRAL})')
        self.send_velocity(self.VEL_NEUTRAL)
        input('Appuyez sur le bouton de l\'ESC pour enregistrer NEUTRE, puis appuyez sur ENTRÉE...')
        
        self.get_logger().info('=== CALIBRATION TERMINÉE ===')
        self.get_logger().info('L\'ESC devrait émettre un bip de confirmation')
        
    def test_sequence(self):
        """Séquence de test après calibration"""
        self.get_logger().info('=== TEST DE L\'ESC ===')
        
        # Neutre
        self.get_logger().info('Position neutre...')
        self.send_velocity(self.VEL_NEUTRAL)
        time.sleep(2)
        
        # Léger avant
        self.get_logger().info('Léger avant (0.3)...')
        self.send_velocity(0.3)
        time.sleep(2)
        
        # Retour neutre
        self.get_logger().info('Retour neutre...')
        self.send_velocity(self.VEL_NEUTRAL)
        time.sleep(2)
        
        # Léger arrière
        self.get_logger().info('Léger arrière (-0.3)...')
        self.send_velocity(-0.3)
        time.sleep(2)
        
        # Retour neutre
        self.get_logger().info('Retour neutre final...')
        self.send_velocity(self.VEL_NEUTRAL)
        
        self.get_logger().info('=== TEST TERMINÉ ===')


def main(args=None):
    rclpy.init(args=args)
    
    esc_node = ESCCalibration()
    
    print("\n" + "="*50)
    print("CALIBRATION ESC TAMIYA TBLE-04S")
    print("="*50)
    print("\nOptions:")
    print("1 - Calibration complète")
    print("2 - Test de l'ESC")
    print("3 - Envoi manuel de vitesse")
    print("="*50 + "\n")
    
    choice = input("Votre choix (1-3): ")
    
    if choice == '1':
        input("\nBranchez l'ESC et appuyez sur ENTRÉE pour commencer la calibration...")
        esc_node.calibrate()
    elif choice == '2':
        esc_node.test_sequence()
    elif choice == '3':
        try:
            vel_value = float(input("Entrez la vitesse (-1.0 à 1.0): "))
            if -1.0 <= vel_value <= 1.0:
                esc_node.send_velocity(vel_value)
            else:
                print("Valeur hors limites!")
        except ValueError:
            print("Valeur invalide!")
    
    print("\nAppuyez sur Ctrl+C pour quitter...")
    try:
        rclpy.spin(esc_node)
    except KeyboardInterrupt:
        pass
    
    esc_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
