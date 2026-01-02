import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

// Enhanced Lesson header with visual depth and hierarchy
export default function LessonHeader({ title, description, children }) {
  return (
    <header className={clsx(styles.lessonHeader)}>
      <div className={styles.lessonHeaderContent}>
        <h1 className={styles.lessonTitle}>{title}</h1>
        {description && <p className={styles.lessonDescription}>{description}</p>}
        {children}
      </div>
      <div className={styles.lessonHeaderDecoration}></div>
    </header>
  );
}