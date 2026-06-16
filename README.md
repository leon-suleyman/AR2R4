# AR2R4

![Foto del robot AR2R4 con y sin la tapa puesta](imagenes/ar2-r4.png)

El AR2R4 es un robot educativo de diseño abierto basado en
la placa Arduino UNO R4, orientado específicamente a la
enseñanza progresiva de programación y robótica móvil.
El robot fue concebido con el objetivo de ofrecer una
plataforma de muy bajo costo (se estima que el costo total es
USD 120, a precios de mayo de 2026), fácilmente reproducible
mediante impresión 3D y componentes comerciales accesibles,
pero al mismo tiempo suficientemente flexible como para cu-
brir distintos niveles de complejidad educativa y tecnológica.

## Componentes

Esta tabla contiene todos los componentes necesarios, el modelo utilizado y la cantidad necesaria para armar el AR2R4 cuyo modelo 3D está compartido en este repositorio.

| Componente          | Modelo                          | Cantidad |
| ------------------- | ------------------------------- | -------- |
| Arduino             | UNO R4                          | 1        |
| Puente H            | L298N                           | 1        |
| Sensor óptico       | LM393                           | 2        |
| Sonar               | HC-SR04                         | 3        |
| Módulo IR           | TCRT5000                        | 2        |
| Resistencia         | 10 $\Omega$, 1/4 W              | 3        |
| Capacitor           | 470 $\mu$ F, 16 V                | 1        |
| LED                 | 5mm                             | 3        |
| Motor DC            | 3--6 V, doble eje               | 2        |
| Rueda motriz        | 68x26mm                         | 2        |
| Disco encoder       | 25.5mm, 20 ranuras              | 2        |
| Rueda castor        | 25 mm                           | 1        |
| Breadboard          | 170 puntos                      | 1        |
| Cable Dupont        | 10cm macho-hembra               | 34       |
| Cable Dupont        | 10cm macho-macho                | 13       |
| Cable               | 1.5x100mm                       | 6        |
| Interruptor         | 20mm                            | 1        |
| Tornillo            | 3x12mm auto-perf.               | 12       |
| Tornillo            | 3x12mm cabeza plana             | 2        |
| Tornillo            | 3x30mm cabeza plana             | 4        |
| Tuerca              | 3mm                             | 6        |
| Porta pilas         | 4xAA                            | 2        |
| Pila recargable     | AA, 2700mAh                     | 8        |
| Chasis              | PLA, 250g                       | 1        |

Al ser imprimible, el chasis puede ser modificado desde su modelo 3D y queda abierto a que modifiquen el AR2R4 de la manera que más les interese.

## Circuito

También les dejamos una imagen mostrando todas las conexiones eléctricas de este modelo del AR2R4, para que puedan recrearlo. Tengan en cuenta que los cables más gruesos son para conectar los motores al driver y el driver a las baterías.

![Diagrama de las conexiones de los componentes electrónicos del robot AR2R4](imagenes/AR2R4_circuito.png)