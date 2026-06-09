# Lab-3-Inverse-Dynamics-Control 

## Introducción

El control por movimiento de los manipuladores móviles requieren considerar fuerzas externas al sistema, así como la provocada por la gravedad, inercia, Coriolis, etc. Si no se tienen en cuenta estos efectos, el seguimiento de trayectorias del robot puede provocar errores significativos. En esta práctica se estudiará una técnica para compensar estas dinámicas no lineales del manipulador, el control por dinámica inversa (Inverse Dynamics Control), el cual, nos permitirá linealizar  el sistema, facilitando el diseño de controladores. El informe queda dividido en dos puntos: la compensación de gravedad y la cancelación dinámica completa (además de la fuerza de la gravedad, se compensan los efectos no lineales como la inercia, Coriolis y fricción).

---

## Tarea 1: Compensación de la gravedad

En este ejercicio se solicita la implementación de un sistema que compense la fuerza de la gravedad, permitiendo que el manipulador se mantenga en la posición requerida sin "caerse" por la aceleración que esta provoca. Si se le aplica cualquier otra fuerza externa, el control no la compensaría (Para ello se realizará la tarea 2).

### Fundamentos teóricos

Tal y como se ha explicado previamente, el torque demandado es igual al producido por la fuerza de la gravedad:

$$
M(q)\ddot{q} + C(q,\dot{q})\dot{q} + F_b\dot{q} + g(q) = \tau
$$

Donde:

- $M(q)$ es la matriz de inercia.
- $C(q,\dot{q})\dot{q}$ son los efectos de las fuerzas de Coriolis y centrífugas.
- $F_b\dot{q}$ representa la fuerza viscosa.
- $g(q)$ es el vector gravedad (objetivo a cancelar en este caso).
- $\tau$ es el vector par comandado al manipulador.

Para cancelar la fuerza de gravedad, $\tau$ y $g(q)$ deben ser idénticos:

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

![Resultado de la tarea 1.](images/compensacion_gravedad.png)

Por otra parte, se comprueba con el comando `python3 wrench_trackbar_publisher.py` que al someter el efector final a otras fuerzas externas, el robot es empujado en el sentido de la misma, ignorando su propio peso, por lo que no cae.

![Demostración](images/compensacion_gravedad.gif)

---

## Tarea 2: Cancelación Dinámica Completa

Esta segunda tarea trata de compensar las dinámicas no lineales del manipulador, así como la fuerza centrífuga, Coriolis y fricción). A continuación se detallan los pasos para ello.

### Fundamentos teóricos

Inicialmente, se parte de la anterior ecuación de la dinámica del manipulador:

$$
M(q)\ddot{q} + C(q,\dot{q})\dot{q} + F_b\dot{q} + g(q) = \tau
$$

Donde se busca cancelar toda la dinámica no lineal, por lo que será necesario un par tal que:

$$
\tau = M(q)\ddot{q}_d + C(q,\dot{q})\dot{q} + F_b\dot{q} + g(q) = M(q)\ddot{q}_d + n(q,\dot{q})
$$

Al sustituir esta ley de control en la dinámica, resulta:

$$
M(q)\ddot{q} = M(q)\ddot{q}_d
$$

Y al ser invertible la matriz de inercia:

$$
\ddot{q} = \ddot{q}_d
$$

Otorgando la posición articular deseada. Este esquema de control es el siguiente:

![Bucle Control tarea 2](images/bucle_cancel_dynamics.png)

El cual, para pasar de la aceleración angular a posición, necesita dos integradores, siendo la planta marginalmente estable. Al añadir el control PD explicado previamente en un bucle cerrado, resulta un sistema de segundo orden, el cual, ante entrada escalón, cuenta con error nulo.

Para implementar este control se modifica el método `cancel_dynamics()` con la dinámica proporcionada en el nodo `uma_arm_dynamics.cpp` y el cálculo del torque explicado anteriormente:

```cpp

// Method to calculate joint acceleration
Eigen::VectorXd cancel_dynamics()
{
    // Initialize M, C, Fb, g_vec, and tau_ext
    Eigen::MatrixXd M(2, 2);
    Eigen::VectorXd C(2);
    Eigen::MatrixXd Fb(2, 2);
    Eigen::VectorXd g_vec(2);
    Eigen::MatrixXd J(2, 2);
    Eigen::VectorXd tau_ext(2);

    // Initialize q1, q2, q_dot1, and q_dot2
    double q1 = joint_positions_(0);
    double q2 = joint_positions_(1);
    double q_dot1 = joint_velocities_(0);
    double q_dot2 = joint_velocities_(1);

    // Calculate matrix M
    M(0, 0) = m1_ * pow(l1_, 2) + m2_ * (pow(l1_, 2) + 2 * l1_ * l2_ * cos(q2) + pow(l2_, 2));
    M(0, 1) = m2_ * (l1_ * l2_ * cos(q2) + pow(l2_, 2));
    M(1, 0) = M(0, 1);
    M(1, 1) = m2_ * pow(l2_, 2);

    // Calculate vector C (C is 2x1 because it already includes q_dot)
    C << -m2_ * l1_ * l2_ * sin(q2) * (2 * q_dot1 * q_dot2 + pow(q_dot2, 2)),
    m2_ * l1_ * l2_ * pow(q_dot1, 2) * sin(q2);

    // Calculate Fb matrix
    Fb << b1_, 0.0,
    0.0, b2_;

    // Calculate g_vec
    g_vec << (m1_ + m2_) * l1_ * g_ * cos(q1) + m2_ * g_ * l2_ * cos(q1 + q2),
        m2_ * g_ * l2_ * cos(q1 + q2);

    // Calculate control torque using the dynamic model: torque = M * q_ddot + C * q_dot + Fb * q_dot + g
    Eigen::VectorXd torque(2);
    Eigen::VectorXd q_dot(2);
    q_dot << q_dot1, q_dot2;

    torque << M * desired_joint_accelerations_ + C + Fb * q_dot + g_vec;

    return torque;
}

```

Posteriormente, se crea su correspondiente `dynamics_cancellation_launch.py` y se añade en el `CMakeLists.txt`.

---

### Resultados



---

### Conclusiones:

