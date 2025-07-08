import math

def calcular_angulos_pierna(x, y, z):
    """
    Calcula los ángulos de las articulaciones de una pierna de gato robot.
    
    Parámetros:
    x, y, z: coordenadas del punto objetivo en milímetros (mm)
    
    Retorna:
    Tupla con los ángulos (coxa, femur, tibia) en grados
    
    Nota: Todas las constantes están en mm: 1600, 40, 120, 28800
    """
    
    try:
        # Cálculo del ángulo COXA
        # coxa = tan^-1(z/y) + tan^-1(sqrt((z² + y²)-1600)/40)
        # El 40 está en el denominador, FUERA de la raíz cuadrada
        parte1_coxa = math.atan(z / y)
        
        # Verificar que el argumento de la raíz sea positivo
        arg_raiz_coxa = (z**2 + y**2) - 1600
        if arg_raiz_coxa < 0:
            raise ValueError(f"Error en coxa: argumento de raíz negativo: {arg_raiz_coxa}")
        
        parte2_coxa = math.atan(math.sqrt(arg_raiz_coxa) / 40)
        coxa_rad = parte1_coxa + parte2_coxa
        coxa_deg = math.degrees(coxa_rad)
        
        # Cálculo del ángulo TIBIA (se calcula antes porque se necesita para fémur)
        # tibia = cos^-1(((z²+y²)-1600+x²-28800)/-28800)
        arg_acos_tibia = ((z**2 + y**2) - 1600 + x**2 - 28800) / (-28800)
        
        # Verificar que el argumento esté en el rango [-1, 1]
        if arg_acos_tibia < -1 or arg_acos_tibia > 1:
            raise ValueError(f"Error en tibia: argumento de arccos fuera de rango: {arg_acos_tibia}")
        
        tibia_rad = math.acos(arg_acos_tibia)
        tibia_deg = math.degrees(tibia_rad)
        
        # Cálculo del ángulo FEMUR
        # femur = tan^-1(x/(sqrt((z² + y²)-1600))) + sen^-1(120*sen(tibia)/sqrt(((z²+y²)-1600)+x²))
        # Denominador confirmado: sqrt(((z²+y²)-1600)+x²)
        
        # Primera parte: tan^-1(x/(sqrt((z² + y²)-1600)))
        arg_raiz_femur1 = (z**2 + y**2) - 1600
        if arg_raiz_femur1 <= 0:
            raise ValueError(f"Error en femur: argumento de raíz no positivo: {arg_raiz_femur1}")
        
        parte1_femur = math.atan(x / math.sqrt(arg_raiz_femur1))
        
        # Segunda parte: sen^-1(120*sen(tibia)/sqrt(((z²+y²)-1600)+x²))
        arg_raiz_femur2 = ((z**2 + y**2) - 1600) + x**2
        if arg_raiz_femur2 <= 0:
            raise ValueError(f"Error en femur: argumento de raíz no positivo: {arg_raiz_femur2}")
        
        arg_asin_femur = (120 * math.sin(tibia_rad)) / math.sqrt(arg_raiz_femur2)
        
        # Verificar que el argumento esté en el rango [-1, 1]
        if arg_asin_femur < -1 or arg_asin_femur > 1:
            raise ValueError(f"Error en femur: argumento de arcsen fuera de rango: {arg_asin_femur}")
        
        parte2_femur = math.asin(arg_asin_femur)
        femur_rad = parte1_femur + parte2_femur
        femur_deg = math.degrees(femur_rad)
        
        return coxa_deg, femur_deg, tibia_deg
        
    except ZeroDivisionError as e:
        print(f"Error: División por cero - {e}")
        return None
    except ValueError as e:
        print(f"Error: {e}")
        return None
    except Exception as e:
        print(f"Error inesperado: {e}")
        return None

def main():
    """
    Función principal que solicita las coordenadas y calcula los ángulos
    """
    print("=== Calculadora de Ángulos para Pierna de Gato Robot ===")
    print()
    
    try:
        # Solicitar coordenadas al usuario
        x = float(input("Ingresa el valor de x (en mm): "))
        y = float(input("Ingresa el valor de y (en mm): "))
        z = float(input("Ingresa el valor de z (en mm): "))
        
        print(f"\nCoordenadas ingresadas: x={x}mm, y={y}mm, z={z}mm")
        print("\nCalculando ángulos...")
        
        # Calcular los ángulos
        resultado = calcular_angulos_pierna(x, y, z)
        
        if resultado is not None:
            coxa, femur, tibia = resultado
            
            print("\n=== RESULTADOS ===")
            print(f"Ángulo COXA:  {coxa:.4f}°")
            print(f"Ángulo FEMUR: {femur:.4f}°")
            print(f"Ángulo TIBIA: {tibia:.4f}°")
            
            # Mostrar también en radianes
            print(f"\nEn radianes:")
            print(f"Ángulo COXA:  {math.radians(coxa):.4f} rad")
            print(f"Ángulo FEMUR: {math.radians(femur):.4f} rad")
            print(f"Ángulo TIBIA: {math.radians(tibia):.4f} rad")
        else:
            print("No se pudieron calcular los ángulos debido a errores en las ecuaciones.")
            
    except ValueError:
        print("Error: Por favor ingresa valores numéricos válidos.")
    except KeyboardInterrupt:
        print("\nPrograma interrumpido por el usuario.")

if __name__ == "__main__":
    main()