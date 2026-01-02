import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

// Enhanced Lesson container with clear boundaries
export default function LessonContainer({
  children,
  className = '',
  title,
  subtitle,
  icon = '📚',
  type = 'default', // 'default', 'featured', 'important', 'interactive'
  tags,
  progress,
  badge,
  badgeType, // 'new', 'updated', 'coming-soon'
  number,
  ...props
}) {
  const containerClasses = clsx(
    styles.lessonContainer,
    styles[`lessonContainer--${type}`],
    className
  );

  const badgeClasses = clsx(
    styles.lessonBadge,
    badgeType && styles[`lessonBadge--${badgeType}`]
  );

  return (
    <div className={containerClasses} {...props}>
      {badge && (
        <div className={badgeClasses}>
          {badge}
        </div>
      )}
      <div className={styles.lessonDecoration}></div>

      <div className={styles.lessonHeader}>
        <h2 className={styles.lessonTitle}>
          {number && <span className={styles.lessonNumber}>{number}</span>}
          <span className={styles.lessonIcon}>{icon}</span>
          {title}
        </h2>
        {subtitle && <p className={styles.lessonSubtitle}>{subtitle}</p>}
      </div>

      <div className={styles.lessonContent}>
        {children}
      </div>

      {(tags || progress) && (
        <div className={styles.lessonFooter}>
          {tags && (
            <div className={styles.lessonTags}>
              {tags.map((tag, index) => (
                <span key={index} className={styles.lessonTag}>
                  {tag}
                </span>
              ))}
            </div>
          )}
          {progress && (
            <div className={styles.lessonProgress}>
              <div
                className={styles.lessonProgressFill}
                style={{ width: `${progress}%` }}
              ></div>
            </div>
          )}
        </div>
      )}
    </div>
  );
}