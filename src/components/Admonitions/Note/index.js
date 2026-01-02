import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

// Enhanced Note admonition with new visual design
export default function Note({ children, title }) {
  return (
    <div className={clsx('alert alert--secondary', styles.note)}>
      <div className={styles.noteHeader}>
        <span className={styles.noteIcon}>📘</span>
        <h5 className={styles.noteTitle}>{title || 'Note'}</h5>
      </div>
      <div className={styles.noteContent}>
        {children}
      </div>
    </div>
  );
}