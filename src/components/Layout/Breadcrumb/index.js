import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import { useLocation } from '@docusaurus/router';
import styles from './styles.module.css';

// Enhanced breadcrumb with depth and visual hierarchy
export default function Breadcrumb({ className = '', ...props }) {
  const location = useLocation();

  // Simple breadcrumb based on URL path
  const pathSegments = location.pathname
    .split('/')
    .filter(segment => segment && segment !== 'docs')
    .map(segment => segment.replace(/-/g, ' '));

  const breadcrumbs = pathSegments.map((segment, index) => {
    const path = '/' + location.pathname.split('/').slice(1, index + 2).join('/');
    const isLast = index === pathSegments.length - 1;

    return {
      label: segment.charAt(0).toUpperCase() + segment.slice(1),
      path: isLast ? null : path,
      isLast
    };
  });

  return (
    <nav className={clsx(styles.breadcrumb, className)} {...props}>
      <ol className={styles.breadcrumbList}>
        <li className={styles.breadcrumbItem}>
          <Link to="/" className={styles.breadcrumbLink}>
            Home
          </Link>
        </li>
        {breadcrumbs.map((breadcrumb, index) => (
          <li key={index} className={styles.breadcrumbItem}>
            <span className={styles.breadcrumbSeparator}>›</span>
            {breadcrumb.path ? (
              <Link to={breadcrumb.path} className={styles.breadcrumbLink}>
                {breadcrumb.label}
              </Link>
            ) : (
              <span className={clsx(styles.breadcrumbLink, styles.breadcrumbCurrent)}>
                {breadcrumb.label}
              </span>
            )}
          </li>
        ))}
      </ol>
    </nav>
  );
}