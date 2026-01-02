import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

export default function CTASection() {
  return (
    <section className={styles.cta}>
      <div className="container">
        <div className="row">
          <div className="col col--12 text--center">
            <h2>Ready to dive into Physical AI & Humanoid Robotics?</h2>
            <p className="padding-horiz--md">
              Start your learning journey with our comprehensive textbook covering everything from foundational concepts to advanced implementations.
            </p>
            <div className={styles.buttons}>
              <Link
                className="button button--primary button--lg"
                to="/docs/chapters/foundations-of-physical-ai/lesson-1.1">
                Begin Learning Now
              </Link>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}