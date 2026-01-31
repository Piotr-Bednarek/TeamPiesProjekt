"""
LQR Calculator Module
Oblicza wzmocnienia K dla regulatora LQR (model 3-stanowy dla Ball on Beam)
"""

import numpy as np
from scipy.linalg import solve_continuous_are


def compute_lqr_gains(Q_x: float, Q_v: float, Q_theta: float, R: float, T_servo: float = 0.095, g: float = 9.81):
    """
    Oblicza wzmocnienia K dla regulatora LQR.
    
    Model 3-stanowy:
    - Stan x = [pozycja_kulki, prędkość_kulki, kąt_belki]
    - Sterowanie u = żądany kąt serwa
    
    Args:
        Q_x: Waga dla błędu pozycji (większa = szybsza reakcja na błąd pozycji)
        Q_v: Waga dla prędkości (większa = większe tłumienie oscylacji)
        Q_theta: Waga dla kąta belki (większa = mniejsze wychylenia serwa)
        R: Koszt sterowania (większa = wolniejsza reakcja, mniejsze zużycie serwa)
        T_servo: Stała czasowa serwa [s] (domyślnie 0.1)
        g: Przyspieszenie grawitacyjne [m/s^2]
        
    Returns:
        tuple: (K1, K2, K3) - wzmocnienia regulatora LQR
            K1 - wzmocnienie dla pozycji
            K2 - wzmocnienie dla prędkości  
            K3 - wzmocnienie dla kąta belki
    """
    c_ball = (3.0 / 5.0) * g  # ~5.9 m/s^2/rad (dla toczącej się kulki)
    
    # Macierz stanu A
    A = np.array([
        [0.0, 1.0, 0.0],           # dx/dt = v
        [0.0, 0.0, -c_ball],       # dv/dt = -c * theta
        [0.0, 0.0, -1.0/T_servo]   # dtheta/dt = -theta/T + u/T
    ])
    
    # Macierz wejścia B
    B = np.array([
        [0.0],
        [0.0],
        [1.0/T_servo]
    ])
    
    # Macierz wag Q (koszt stanów)
    Q = np.diag([Q_x, Q_v, Q_theta])
    
    # Macierz wag R (koszt sterowania)
    R_mat = np.array([[R]])
    
    # Rozwiązanie równania Riccatiego
    P = solve_continuous_are(A, B, Q, R_mat)
    
    # Obliczenie wzmocnień K = R^-1 * B^T * P
    K = np.linalg.inv(R_mat) @ B.T @ P
    
    # Zwracamy wartości bezwzględne (znaki obsługuje STM)
    K1 = abs(K[0, 0])
    K2 = abs(K[0, 1])
    K3 = abs(K[0, 2])
    
    return K1, K2, K3


def validate_lqr_params(Q_x: float, Q_v: float, Q_theta: float, R: float, T_servo: float) -> tuple:
    """
    Sprawdza poprawność parametrów LQR.
    
    Returns:
        tuple: (is_valid: bool, error_message: str or None)
    """
    if Q_x <= 0 or Q_v < 0 or Q_theta < 0:
        return False, "Wagi Q muszą być dodatnie (Q_x > 0, Q_v >= 0, Q_theta >= 0)"
    
    if R <= 0:
        return False, "Waga R musi być dodatnia (R > 0)"
    
    if T_servo <= 0 or T_servo > 1.0:
        return False, "Stała czasowa serwa T musi być w zakresie (0, 1] sekundy"
    
    return True, None


def apply_friction_compensation(control_deg: float, 
                                 min_angle_to_move: float = 2.0,
                                 control_threshold: float = 0.1) -> float:
    """
    Kompensacja tarcia statycznego (Stiction).
    
    Jeśli LQR chce ruszyć kulką (control_deg nie jest zerem), dodaj minimalny
    kąt potrzebny do przełamania oporów tarcia statycznego druku 3D.
    
    Args:
        control_deg: Wyjście LQR przed kompensacją [stopnie]
        min_angle_to_move: Minimalny kąt do pokonania tarcia [stopnie, domyślnie 2°]
        control_threshold: Próg wykrywania intencji ruchu [stopnie, domyślnie 0.1°]
        
    Returns:
        float: Skompensowany kąt sterowania [stopnie]
    """
    friction_comp = 0.0
    
    if control_deg > control_threshold:
        friction_comp = min_angle_to_move
    elif control_deg < -control_threshold:
        friction_comp = -min_angle_to_move
    
    return control_deg + friction_comp
