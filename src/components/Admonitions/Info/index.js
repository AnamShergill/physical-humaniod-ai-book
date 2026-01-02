import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

// Enhanced Info admonition with new visual design
export default function Info({ children, title }) {
  return (
    <div className={clsx('alert alert--info', styles.info)}>
      <div className={styles.infoHeader}>
        <span className={styles.infoIcon}>ℹ️</span>
        <h5 className={styles.infoTitle}>{title || 'Info'}</h5>
      </div>
      <div className={styles.infoContent}>
        {children}
      </div>
    </div>
  );
}