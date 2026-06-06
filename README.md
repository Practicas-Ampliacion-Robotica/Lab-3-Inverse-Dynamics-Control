# Lab-3-Inverse-Dynamics-Control 

## Introducción

El control por movimiento de los manipuladores móviles requieren considerar fuerzas externas al sistema, así como la provocada por la gravedad, inercia, Coriolis, etc. Si no se tienen en cuenta estos efectos, el seguimiento de trayectorias del robot puede provocar errores significativos. En esta práctica se estudiará una técnica para compensar estas dinámicas no lineales del manipulador, el control por dinámica inversa (Inverse Dynamics Control), el cual, nos permitirá linealizar  el sistema, facilitando el diseño de controladores. El informe queda dividido en dos puntos: la compensación de gravedad y la cancelación dinámica completa (además de la fuerza de la gravedad, se compensan los efectos de la inercia, Coriolis y fricción).

---

## Tarea 1: Compensación de la gravedad

En este ejercicio se solicita la implementación de un sistema que compense la fuerza de la gravedad, permitiendo que el manipulador se mantenga en la posición requerida sin "caerse" por la aceleración que esta provoca. Si se le aplica cualquier otra fuerza externa, el control no la compensaría (Para ello se realizará la tarea 2).

### Fundamentos teóricos

Tal y como se ha explicado previamente, el torque demandado es igual al producido por la fuerza de la gravedad:

$$
M(q)\ddot{q} + C(q,\dot{q})\dot{q} + F_b\dot{q} + g(q) = \tau
$$

donde \(g(q)\) es el vector gravedad, por lo tanto:

$$
\tau = g(q)
$$

Por ello, se crea un nodo ROS2 llamado `gravity_compensation.cpp` (con su correspondiente `gravity_compensation_launch.py` y modificación del `CMakeLists.txt`):

```cpp
// Method to calculate the desired joint torques
Eigen::VectorXd gravity_compensation()
{
    // Placeholder for calculate the commanded torques
    // Calculate the control torque to compensate only for gravity effects: tau = g(q)

    // Calculate g_vect

    // Initialize q1, q2, q_dot1, and q_dot2
    double q1 = joint_positions_(0);
    double q2 = joint_positions_(1);
    Eigen::VectorXd g_vec(2);

    g_vec << (m1_ + m2_) * l1_ * g_ * cos(q1) + m2_ * g_ * l2_ * cos(q1 + q2),
        m2_ * g_ * l2_ * cos(q1 + q2);

    // // Calculate desired torque
    Eigen::VectorXd torque(2);
    torque << g_vec;

    return torque;
}
```

En él, se han tomado las dos variables articulares, $q1$ y $q2$ para construir con ellas el torque necesario según la siguiente fórmula:

$$
g(q)=
\begin{bmatrix}
(m_1+m_2)l_1g\cos(q_1)+m_2l_2g\cos(q_1+q_2) \\
m_2l_2g\cos(q_1+q_2)
\end{bmatrix}
$$

---

### Resultados

Al enviar (45º, 45º) como consigna de las variables articulares se obtiene el siguiente comportamiento del manipulador:

FOTO 1

![Resultado de la tarea 1.](images/tarea1.png)

Por otra parte, se comprueba que al someter el efector final a otras fuerzas externas, el robot es empujado en el sentido de la misma, ignorando su propio peso, por lo que no cae.

VIDEO 1

FOTOS GRAFICOS

---

## Tarea 2: Cancelación Dinámica Completa

### Fundamentos teóricos



---

### Resultados

---

### Conclusiones:

