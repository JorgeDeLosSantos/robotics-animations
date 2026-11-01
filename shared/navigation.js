/** Global navigation for independent robotics labs (no framework required). */
const sections = [
  { name: 'Transformations', items: [
    ['Homogeneous Transformations', 'transformations/homogeneous-transformations.html'],
    ['Euler Angles', 'transformations/euler-angles.html'],
  ] },
  { name: 'Kinematics', items: [
    ['Forward Kinematics', 'kinematics/forward-kinematics.html'],
    ['Workspace', 'kinematics/workspace.html'],
    ['Jacobian Matrix', 'kinematics/jacobian-matrix.html'],
    ['Position Inverse Kinematics', 'kinematics/inverse-kinematics-position.html'],
    ['Full-Pose Inverse Kinematics', 'kinematics/inverse-kinematics-6dof.html'],
  ] },
  { name: 'Trajectories', items: [
    ['Joint-Space Trajectories', 'trajectories/joint-space.html'],
  ] },
];
const root = new URL('../', import.meta.url);
const nav = document.createElement('nav');
nav.className = 'robotics-nav';
nav.setAttribute('aria-label', 'Laboratory navigation');

const home = document.createElement('a');
home.href = new URL('index.html', root).href;
home.className = 'robotics-nav-home';
home.textContent = 'Robotics Lab';
nav.append(home);

const themeToggle = document.createElement('button');
themeToggle.type = 'button';
themeToggle.className = 'robotics-theme-toggle';
const refreshThemeToggle = () => {
  const theme = window.RoboticsTheme.getTheme();
  themeToggle.textContent = theme === 'dark' ? '☾ Dark' : '☀ Light';
  themeToggle.setAttribute('aria-label', 'Switch to ' + (theme === 'dark' ? 'light' : 'dark') + ' theme');
  themeToggle.title = themeToggle.getAttribute('aria-label');
};
themeToggle.addEventListener('click', () => {
  window.RoboticsTheme.setTheme(window.RoboticsTheme.getTheme() === 'dark' ? 'light' : 'dark');
});
document.addEventListener('robotics:themechange', refreshThemeToggle);
refreshThemeToggle();

for (const section of sections) {
  const details = document.createElement('details');
  const summary = document.createElement('summary');
  summary.textContent = section.name;
  details.append(summary);

  const menu = document.createElement('div');
  menu.className = 'robotics-nav-menu';
  for (const [name, path] of section.items) {
    const link = document.createElement('a');
    link.href = new URL(path, root).href;
    link.textContent = name;
    if (link.pathname === location.pathname) link.setAttribute('aria-current', 'page');
    menu.append(link);
  }
  details.append(menu);
  nav.append(details);
  document.addEventListener('click', (event) => {
    if (!details.contains(event.target)) details.open = false;
  });
}
nav.addEventListener('keydown', (event) => {
  if (event.key === 'Escape') {
    nav.querySelectorAll('details[open]').forEach((menu) => { menu.open = false; });
  }
});

nav.append(themeToggle);

const mount = document.createElement('div');
mount.className = 'robotics-nav-mount';
mount.append(nav);
document.body.prepend(mount);
const updateHeight = () => {
  document.documentElement.style.setProperty(
    '--robotics-nav-height', mount.getBoundingClientRect().height + 'px'
  );
};
updateHeight();
window.addEventListener('resize', updateHeight);
if (typeof ResizeObserver !== 'undefined') {
  new ResizeObserver(updateHeight).observe(mount);
}
