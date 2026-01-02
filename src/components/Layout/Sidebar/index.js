import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import { useLocation } from '@docusaurus/router';
import styles from './styles.module.css';

// Enhanced sidebar with progress indicators and visual feedback
export default function Sidebar({ sidebar, className }) {
  const location = useLocation();

  return (
    <aside className={clsx('col', 'col--3', styles.sidebar, className)}>
      <nav className={styles.sidebarNav}>
        <ul className={styles.sidebarList}>
          {sidebar.map((item, index) => (
            <li key={index} className={styles.sidebarItem}>
              {item.href ? (
                <Link
                  className={clsx(
                    styles.sidebarLink,
                    location.pathname === item.href && styles.sidebarLinkActive
                  )}
                  to={item.href}
                >
                  {item.label}
                  {location.pathname === item.href && (
                    <span className={styles.sidebarActiveIndicator}>●</span>
                  )}
                </Link>
              ) : (
                <span className={styles.sidebarCategory}>{item.label}</span>
              )}
              {item.items && item.items.length > 0 && (
                <ul className={styles.sidebarSublist}>
                  {item.items.map((subItem, subIndex) => (
                    <li key={subIndex} className={styles.sidebarSubitem}>
                      <Link
                        className={clsx(
                          styles.sidebarLink,
                          styles.sidebarSublink,
                          location.pathname === subItem.href && styles.sidebarLinkActive
                        )}
                        to={subItem.href}
                      >
                        {subItem.label}
                        {location.pathname === subItem.href && (
                          <span className={styles.sidebarActiveIndicator}>●</span>
                        )}
                      </Link>
                    </li>
                  ))}
                </ul>
              )}
            </li>
          ))}
        </ul>
      </nav>
    </aside>
  );
}