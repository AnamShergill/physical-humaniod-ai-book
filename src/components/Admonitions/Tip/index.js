import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

// Enhanced Tip admonition with new visual design
export default function Tip({ children, title }) {
  return (
    <div className={clsx('alert alert--success', styles.tip)}>
      <div className={styles.tipHeader}>
        <span className={styles.tipIcon}>💡</span>
        <h5 className={styles.tipTitle}>{title || 'Tip'}</h5>
      </div>
      <div className={styles.tipContent}>
        {children}
      </div>
    </div>
  );
}