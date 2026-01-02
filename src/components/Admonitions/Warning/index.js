import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

// Enhanced Warning admonition with new visual design
export default function Warning({ children, title }) {
  return (
    <div className={clsx('alert alert--warning', styles.warning)}>
      <div className={styles.warningHeader}>
        <span className={styles.warningIcon}>⚠️</span>
        <h5 className={styles.warningTitle}>{title || 'Warning'}</h5>
      </div>
      <div className={styles.warningContent}>
        {children}
      </div>
    </div>
  );
}