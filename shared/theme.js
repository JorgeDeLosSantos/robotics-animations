/**
 * Shared theme preference for every standalone robotics lab.
 * An explicit choice wins; otherwise follow the OS color scheme.
 * Loaded synchronously from <head> to avoid a flash of the wrong theme.
 */
(() => {
  'use strict';
  const key = 'robotics-lab-theme';
  const media = window.matchMedia?.('(prefers-color-scheme: dark)');
  const valid = value => value === 'dark' || value === 'light';
  const readPreference = () => {
    try { const saved = localStorage.getItem(key); return valid(saved) ? saved : null; }
    catch (_) { return null; }
  };
  const systemTheme = () => media?.matches ? 'dark' : 'light';
  let explicitTheme = readPreference();
  let activeTheme = explicitTheme || systemTheme();

  const update = theme => {
    if (!valid(theme) || theme === activeTheme && document.documentElement.dataset.theme === theme) return;
    activeTheme = theme;
    document.documentElement.dataset.theme = theme;
    document.dispatchEvent(new CustomEvent('robotics:themechange', { detail: { theme } }));
  };
  // Set the attribute immediately, before external styles are applied.
  document.documentElement.dataset.theme = activeTheme;

  window.RoboticsTheme = Object.freeze({
    getTheme: () => activeTheme,
    setTheme: theme => {
      if (!valid(theme)) throw new TypeError('Theme must be "light" or "dark".');
      explicitTheme = theme;
      try { localStorage.setItem(key, theme); } catch (_) { /* Private browsing */ }
      update(theme);
    },
    useSystemTheme: () => {
      explicitTheme = null;
      try { localStorage.removeItem(key); } catch (_) { /* Private browsing */ }
      update(systemTheme());
    },
  });

  media?.addEventListener?.('change', () => {
    if (explicitTheme === null) update(systemTheme());
  });
})();
