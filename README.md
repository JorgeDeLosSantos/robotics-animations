# Laboratorios interactivos de robótica

Colección de visualizaciones y laboratorios didácticos para explorar transformaciones y cinemática de robots seriales directamente en el navegador.

**[Abrir página de laboratorios](https://jorgedelossantos.github.io/robotics-animations/)**

> Los cambios de la rama `dev` se publicarán en el sitio al integrarse en `main`. Los archivos HTML también pueden revisarse directamente desde el repositorio.

## Transformaciones y orientación

- [Transformaciones homogéneas: sistemas fijo y móvil](transformations/homogeneous-transformations.html)
- [Ángulos de Euler: secuencias intrínsecas](transformations/euler-angles.html)

## Cinemática de robots

- [Cinemática directa con parámetros DH](kinematics/forward-kinematics.html)
- [Matriz Jacobiana y velocidad del efector final](kinematics/jacobian-matrix.html)
- [Cinemática inversa de posición: CCD, juntas R y P](kinematics/inverse-kinematics-position.html)
- [Cinemática inversa de pose completa: DLS, 6 GDL](kinematics/inverse-kinematics-6dof.html)

## Convenciones visuales

- **Z es el eje vertical**; el plano XY corresponde al plano horizontal de la cuadrícula.
- Los ejes de referencia siguen el código de colores habitual: X rojo, Y verde, Z azul.
- Los controles orbitales emplean arrastre izquierdo para rotación, arrastre derecho para desplazamiento y rueda para zoom.
- Se utilizan transformaciones DH estándar en los laboratorios de cinemática.
- Salvo que se indique lo contrario, las dimensiones geométricas se expresan en unidades del modelo y los controles angulares en grados.

## Uso y limitaciones

Cada laboratorio es una página HTML independiente que carga Three.js desde un CDN. Las páginas de cinemática comparten el visor `kinematics/shared-robot-viewer.js`; los algoritmos actuales de CCD y DLS se implementan en `kinematics/ik-solvers.js`.

La reproducción del CCD o DLS representa iteraciones del algoritmo de búsqueda, **no una trayectoria dinámica ejecutable por un robot real**. Los métodos de IK no consideran colisiones ni límites mecánicos articulares. El laboratorio DLS de pose completa se limita por ahora a cadenas con seis o más juntas revolutas.

Para probar los archivos localmente, sirve la raíz del repositorio mediante un servidor HTTP estático (las importaciones ES modules no funcionan de manera fiable abriendo los archivos con `file://`). Por ejemplo:

```bash
python -m http.server 8000
```

Y abre `http://localhost:8000/` en el navegador.

> The standalone fixed-frame and body-frame labs were retired in favor of the combined homogeneous-transformations lab.
