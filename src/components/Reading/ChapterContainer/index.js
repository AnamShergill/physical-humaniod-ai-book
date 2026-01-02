import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

// Enhanced Chapter container with book-section styling
export default function ChapterContainer({
  children,
  className = '',
  title,
  subtitle,
  breadcrumb,
  navigation,
  progress,
  ...props
}) {
  return (
    <div className={clsx(styles.chapterContainer, className)} {...props}>
      <div className={styles.chapterDecoration + ' ' + styles['chapterDecoration--top-left']}>
        📘
      </div>
      <div className={styles.chapterDecoration + ' ' + styles['chapterDecoration--bottom-right']}>
        📚
      </div>

      <div className={styles.chapterHeader}>
        {breadcrumb && (
          <div className={styles.chapterBreadcrumb}>
            {breadcrumb}
          </div>
        )}
        {title && <h1 className={styles.chapterTitle}>{title}</h1>}
        {subtitle && <p className={styles.chapterSubtitle}>{subtitle}</p>}
      </div>

      <div className={styles.chapterContent}>
        {progress && (
          <div className={styles.chapterProgress}>
            {progress}
          </div>
        )}
        {children}
      </div>

      {navigation && (
        <div className={styles.chapterNavigation}>
          {navigation}
        </div>
      )}

      <div className={styles.chapterFooter}>
        © {new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook
      </div>
    </div>
  );
}